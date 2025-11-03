#pragma once

#include <memory>

#include <vamp/collision/environment.hh>
#include <vamp/planning/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
//#include <vamp/planning/validate_constraint.hh>
#include <vamp/planning/rrtc_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <chrono>
using namespace std::chrono;
using namespace std;

namespace vamp::planning
{
    template <typename Robot, std::size_t rake, std::size_t resolution>
    struct RRTC
    {
        using Configuration = typename Robot::Configuration;
        static constexpr auto dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot>;

        inline static auto solve(
            const Configuration &start,
            const Configuration &goal,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            return solve(start, std::vector<Configuration>{goal}, environment, settings, rng);
        }

        inline static auto solve(
            const Configuration &start,
            const std::vector<Configuration> &goals,
            const collision::Environment<FloatVector<rake>> &environment,
            const RRTCSettings &settings,
            typename RNG::Ptr rng) noexcept -> PlanningResult<Robot>
        {
            PlanningResult<Robot> result;

            NN<dimension> start_tree;
            NN<dimension> goal_tree;

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

            for (const auto &goal : goals)
            {
                /*if (validate_motion<Robot, rake, resolution>(start, goal, environment))
                {
                    result.path.emplace_back(start);
                    result.path.emplace_back(goal);
                    result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
                    result.iterations = 0;
                    result.size.emplace_back(1);
                    result.size.emplace_back(1);
                    cout << "Terminate early because valid motion from start to goal already exists\n";
                    return result;
                }*/
            }

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

            while (iter++ < settings.max_iterations and free_index < settings.max_samples)
            {
                auto t0 = high_resolution_clock::now();
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

                auto nearest_vector = temp - nearest_configuration;

                bool reach = nearest_distance < settings.range;
                auto extension_vector =
                    (reach) ? nearest_vector : nearest_vector * (settings.range / nearest_distance);
                auto t1 = high_resolution_clock::now();
                profiler_rrtc.setup_before_validate_times.push_back(duration<double, micro>(t1 - t0).count());
                //std::cout << "extension vector is \n";
                bool successful_path = validate_vector<Robot, rake, resolution>(
                        nearest_configuration,
                        extension_vector,
                        (reach) ? nearest_distance : settings.range,
                        environment);
                auto tp = high_resolution_clock::now();
                profiler_rrtc.validate_constraint_vector_times.push_back(duration<double, micro>(tp - t1).count());
                if (successful_path)
                {
                    auto t2 = high_resolution_clock::now();
                    float *new_configuration_index = buffer_index(free_index);
                    auto new_configuration = nearest_configuration + extension_vector;
                    new_configuration.to_array(new_configuration_index);
                    tree_a->insert(NNNode<dimension>{free_index, {new_configuration_index}});

                    parents[free_index] = nearest_node.index;
                    radii[free_index] = std::numeric_limits<float>::max();

                    free_index++;

                    if (settings.dynamic_domain and nearest_radius != std::numeric_limits<float>::max())
                    {
                        radii[nearest_node.index] *= (1 + settings.alpha);
                    }
                    auto t3 = high_resolution_clock::now();
                    profiler_rrtc.insert_projected_to_tree_times.push_back(duration<double, micro>(t3 - t2).count());
                    // Extend to goal tree
                    const auto other_nearest =
                        tree_b->nearest(NNFloatArray<dimension>{new_configuration_index});
                    if (not other_nearest)
                    {
                        continue;
                    }

                    const auto &[other_nearest_node, other_nearest_distance] = *other_nearest;
                    const auto other_nearest_configuration = other_nearest_node.as_vector();
                    std::cout << "other nearest config is " << other_nearest_configuration << "\n";
                        std::cout << "prior config is " << new_configuration << "\n";
                    auto other_nearest_vector = other_nearest_configuration - new_configuration;
                    std::cout << other_nearest_distance / settings.range << " \n";
                    const std::size_t n_extensions = std::ceil(other_nearest_distance / settings.range);
                    const float increment_length = other_nearest_distance / static_cast<float>(n_extensions);
                    auto increment = other_nearest_vector * (1.0F / static_cast<float>(n_extensions));
                    std::cout << "connecting vector is " << increment << "\n";
                    std::size_t i_extension = 0;
                    auto prior = new_configuration;
                    for (; i_extension < n_extensions and
                           free_index < settings.max_samples;
                         ++i_extension)
                    {
                        //To be able to benchmark how long it takes to run validate_vector
                        // Broke off validate_vector away from for loop condition
                        auto tbp = high_resolution_clock::now();
                        if (!validate_vector<Robot, rake, resolution>(
                               prior, increment, increment_length, environment)) {
                                std::cout << "Increment stopped since not valid\n";
                                break;
                               }
                        std::cout << "Continued!\n";
                        auto tap = high_resolution_clock::now();
                        auto t5 = high_resolution_clock::now();
                        auto next = prior + increment;
                        float *next_index = buffer_index(free_index);
                        next.to_array(next_index);
                        tree_a->insert(NNNode<dimension>{free_index, {next_index}});
                        parents[free_index] = free_index - 1;
                        radii[free_index] = std::numeric_limits<float>::max();

                        free_index++;

                        prior = next;
                        auto t6 = high_resolution_clock::now();
                        std::cout << "COUNT!\n";
                        profiler_rrtc.validate_extend_constraint_vector_times.push_back(duration<double, micro>(tap - tbp).count());
                        profiler_rrtc.insertt_extended_to_tree_times.push_back(duration<double, micro>(t6 - t5).count());
                    }

                    if (i_extension == n_extensions)  // connected
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


                        auto t7 = high_resolution_clock::now();
                        profiler_rrtc.full_extend_times.push_back(duration<double, micro>(t7 - t3).count());
                        auto t8 = high_resolution_clock::now();
                        profiler_rrtc.one_iteration_times.push_back(duration<double, micro>(t8 - t0).count());
                        // a bit rdudant, but this makes sure profiler captures last iteration without changing code structure at all
                        break;
                    }
                    auto t7 = high_resolution_clock::now();
                    profiler_rrtc.full_extend_times.push_back(duration<double, micro>(t7 - t3).count());
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
                auto t8 = high_resolution_clock::now();

                profiler_rrtc.one_iteration_times.push_back(duration<double, micro>(t8 - t0).count());
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(start_tree.size());
            result.size.emplace_back(goal_tree.size());
            profiler_rrtc.printReport();
            cout << "Report for RRTC done\n";
            return result;
        }
    };
}  // namespace vamp::planning
