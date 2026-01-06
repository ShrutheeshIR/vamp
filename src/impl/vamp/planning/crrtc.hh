#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate_constraint.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct CRRTC
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            TaskSpaceConstraint<Robot, rake> &constraint,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, constraint, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            TaskSpaceConstraint<Robot, rake> &constraint,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<dimension> start_tree;
            NN<dimension> goal_tree;
            std::vector <Configuration> projected_vector;

            constexpr const std::size_t start_index = 0;

            auto buffer = std::unique_ptr<float, decltype(&free)>(
                vamp::utils::vector_alloc<float, FloatVectorAlignment, FloatVectorWidth>(
                    settings.max_samples * Configuration::num_scalars_rounded),
                &free);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            std::vector<std::size_t> parents(settings.max_samples);
            std::vector<float> radii(settings.max_samples);

            auto start_time = std::chrono::steady_clock::now();

            // for (const auto &goal : goals)
            // {
            //     if (project_constraint_motion<Robot, rake, resolution>(start, goal, projected_vector, constraint, environment))
            //     {
            //         result.path.emplace_back(start);
            //         result.path.emplace_back(goal);
            //         result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            //         result.iterations = 0;
            //         result.size.emplace_back(1);
            //         result.size.emplace_back(1);

            //         return result;
            //     }
            // }

            // trees
            bool tree_a_is_start = not settings.start_tree_first;
            auto *tree_a = (settings.start_tree_first) ? &goal_tree : &start_tree;
            auto *tree_b = (settings.start_tree_first) ? &start_tree : &goal_tree;

            std::size_t iter = 0;
            std::size_t free_index = start_index + 1;

            // add start to tree
            start.to_array(buffer_index(start_index));
            start_tree.insert(NNNode<dimension>{start_index, {buffer_index(start_index)}});
            parents[start_index] = start_index;
            radii[start_index] = std::numeric_limits<float>::max();

            for (const auto &goal : goals)
            {
                goal.to_array(buffer_index(free_index));
                goal_tree.insert(NNNode<dimension>{free_index, {buffer_index(free_index)}});
                parents[free_index] = free_index;
                radii[free_index] = std::numeric_limits<float>::max();
                free_index++;
            }

            bool connected = false;
            while (iter++ < settings.max_iterations and free_index < settings.max_samples and not connected)
            {
                // if (iter % 1 == 0)
                    // std::cout << "Starting iteration : " << iter << ", " << free_index << ", " <<connected << " , " << settings.max_samples << std::endl;
                float asize = tree_a->size();
                float bsize = tree_b->size();
                float ratio = std::abs(asize - bsize) / asize;

                if ((not settings.balance) or ratio < settings.tree_ratio)
                {
                    std::swap(tree_a, tree_b);
                    tree_a_is_start = not tree_a_is_start;
                }

                auto temp = rng->next();
                typename Robot::ConfigurationBuffer temp_array;
                temp.to_array(temp_array.data());

                const auto nearest = tree_a->nearest(NNFloatArray<dimension>{temp_array.data()});
                if (not nearest)
                {
                    continue;
                }

                const auto &[nearest_node, nearest_distance] = *nearest;
                const auto nearest_radius = radii[nearest_node.index];

                if (settings.dynamic_domain and nearest_radius < nearest_distance)
                {
                    continue;
                }

                const auto nearest_configuration = nearest_node.as_vector();
                //std::cout << "temp is " << temp << " and nearest config is " << nearest_configuration << "\n";
                auto nearest_vector = temp - nearest_configuration;

                bool reach = nearest_distance < settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (settings.range / nearest_distance);
                //std::cout << extension_vector << "\n";
                if (project_constraint_vector<Robot, rake, resolution>(
                        nearest_configuration,
                        extension_vector,
                        (reach) ? nearest_distance : settings.range,
                        projected_vector,
                        constraint,
                        environment,
                        static_cast<ProjMethod>(settings.projection_method),
                        settings.descend_rate
                    ))

                {

                    float *new_configuration_index;
                    Configuration new_configuration;
                    auto parent_index = nearest_node.index;

                    for(auto proj_vector : projected_vector)
                    {
                        new_configuration_index = buffer_index(free_index);
                        new_configuration = proj_vector;
                        new_configuration.to_array(new_configuration_index);
                        tree_a->insert(NNNode<dimension>{free_index, {new_configuration_index}});

                        parents[free_index] = parent_index;
                        radii[free_index] = std::numeric_limits<float>::max();


                        if (settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                        {
                            radii[parent_index] *= (1 + settings.alpha);
                        }

                        parent_index = free_index;
                        free_index++;

                    }


                    auto prior = new_configuration;
                    auto prior_index = new_configuration_index;

                    auto counter = 0U;

                    // Added new feature to upper bound the number of iterations connecting tree A with tree B
                    const auto other_nearest =
                        tree_b->nearest(NNFloatArray<dimension>{new_configuration_index});
                    if (not other_nearest)
                    {
                        continue;
                    }
                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const std::size_t n_extensions = std::ceil(other_nearest_distance / settings.range);
                    auto max_iterations = 2 * n_extensions;
                    //std::cout << "distance is " << other_nearest_distance << "\n";
                    //std::cout << "max iterations is " << max_iterations << "\n";
                    //auto max_iterations = 2;
                    // try to connect to goal directly
                    while (not connected)
                    {   
                        counter++;
                        if (counter > max_iterations)
                            break;
                        // Extend to goal tree
                        const auto other_nearest =
                            tree_b->nearest(NNFloatArray<dimension>{prior_index});
                        if (not other_nearest)
                            break;


                        const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                        const auto other_nearest_configuration = other_nearest_node.as_vector();
                        //std::cout << "nearest neighbor is " << other_nearest_configuration << "\n";
                        auto other_nearest_vector = other_nearest_configuration - prior;
                        bool other_reach = other_nearest_distance < settings.range;

                        auto other_extension_vector =
                            (other_reach) ? other_nearest_vector : other_nearest_vector * (settings.range / other_nearest_distance);

                        if(not project_constraint_vector<Robot, rake, resolution>(
                            prior,
                            other_extension_vector,
                            (other_reach) ? other_nearest_distance : settings.range,
                            projected_vector,
                            constraint,
                            environment,
                            static_cast<ProjMethod>(settings.projection_method),
                            settings.descend_rate
                            )
                        )
                        {
                            break;
                        }
                        if (free_index >= settings.max_samples)
                            break;


                        float *next_index;
                        Configuration next;

                        for(auto proj_vector : projected_vector)
                        {
                            next_index = buffer_index(free_index);
                            next = proj_vector;
                            next.to_array(next_index);
                            tree_a->insert(NNNode<dimension>{free_index, {next_index}});
                            parents[free_index] = free_index - 1;
                            radii[free_index] = std::numeric_limits<float>::max();
                            free_index++;
                        }

                        bool other_reached = (other_nearest_configuration - next).squared_l2_norm() < 0.01;
                        if (other_reached)  // connected
                        {
                            auto current = free_index - 1;
                            result.path.emplace_back(buffer_index(current));
                            while (parents[current] != current)
                            {
                                auto parent = parents[current];
                                result.path.emplace_back(buffer_index(parent));
                                result.cost += result.path[result.path.size() - 1].distance(
                                    result.path[result.path.size() - 2]);
                                current = parent;
                            }

                            std::reverse(result.path.begin(), result.path.end());
                            current = other_nearest_node.index;

                            while (parents[current] != current)
                            {
                                auto parent = parents[current];
                                result.path.emplace_back(buffer_index(parent));
                                result.cost += result.path[result.path.size() - 1].distance(
                                    result.path[result.path.size() - 2]);
                                current = parent;
                            }

                            if (not tree_a_is_start)
                            {
                                std::reverse(result.path.begin(), result.path.end());
                            }
                            connected = true;

                            break;
                        }

                        prior = next;
                        prior_index = next_index;

                    }
                }
                else if (settings.dynamic_domain)
                {
                    if (nearest_radius == std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] = settings.radius;
                    }
                    else
                    {
                        radii[nearest_node.index] =
                            std::max(radii[nearest_node.index] * (1.F - settings.alpha), settings.min_radius);
                    }
                }
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
            std::cout << "Terminated with  : " << iter << ", " << free_index << ", " <<connected << " , " << settings.max_samples << ", " << tree_a->size() << ", " << tree_b->size() << ", " << result.iterations << ", " << result.nanoseconds/1e6 << std::endl;
            return result;
        }
    };
}  // namespace vamp::planning
