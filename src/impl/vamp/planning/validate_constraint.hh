#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate.hh>


namespace vamp::planning
{
    static int invalid_distance_counter_outside = 0;
    static int invalid_distance_counter_inside = 0;
    static int unable_to_project_counter = 0;
    static int collision_counter = 0;
    static int unable_to_project_inside_counter = 0;
    static int collision_inside_counter = 0;

    template <std::size_t rake, std::size_t dimension>
    inline constexpr auto inter_lane_distance_block(const vamp::FloatVector<rake, dimension> &block,
                                                    const vamp::FloatVector<dimension> &start) -> vamp::FloatVector<rake, dimension>
    {
        auto out = block;
        for (std::size_t i = 0; i < dimension; ++i)
        {
            auto svec = start.broadcast(i);
            out[i] = out[i].inter_lane_distance(svec[{0, 0}]);
        }
        return out;
    }

    // template <std::size_t rake, std::size_t dimension>
    // inline constexpr auto inter_lane_distance_block(const vamp::FloatVector<rake, dimension> &block,
    //                                                 const vamp::FloatVector<dimension> &start) -> vamp::FloatVector<rake, dimension>
    // {
    //     auto out = block;
    //     for (std::size_t i = 0; i < dimension; ++i)
    //     {
    //         // Avoid element(i) on flattened buffers by using broadcast, which is shape-safe
    //         const auto start_vec = start.broadcast(i);
    //         out[i] = out[i].S::template inter_lane_distance<0>(out[i].d()->data[0], start_vec.d()->data[0]);
    //     }
    //     return out;
    // }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline constexpr auto project_constraint_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        std::vector<typename Robot::Configuration> &projected_vector,
        ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const collision::Environment<FloatVector<rake>> &environment,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0) -> bool
    {
        projected_vector.clear();

        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;
        typename Robot::template ConfigurationBlock<rake> projected_block;
        typename Robot::template ConfigurationBlock<rake> initial_projected_block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        // std::cout << "Proj method " << projection_method << std::endl;

        bool ableToProject = constraint.projectConfiguration(block, initial_projected_block, projection_method, distance, projection_descent_rate);
        if (not ableToProject)
        {
            // std::cout << "Unable to project " << std::endl;
            unable_to_project_counter++;
            return ableToProject;
        }



        // float max_inter_dist = 0.F;

        auto shifted_block = inter_lane_distance_block(initial_projected_block, start);
        auto inter_rake_distance = shifted_block[0] * shifted_block[0];
        for (auto dim = 1U; dim < Robot::dimension; dim++)
        {
            inter_rake_distance = inter_rake_distance + shifted_block[dim] * shifted_block[dim];
            if (inter_rake_distance.test_any_greater(4 * (distance / rake) * (distance / rake)))
            {
                invalid_distance_counter_outside++;
                // std::cout << "Invalid config due to distance constraint" << inter_rake_distance <<  " with max distance allowed is " << 4 * (distance / rake) * (distance / rake) << std::endl;
                return false;
            }

        }
        // auto max_inter_dist = inter_rake_distance.sqrt();

        // Compute inter-rake differences
        // std::array<vamp::FloatT, Robot::dimension * rake> diff_arr;
        // for (auto i = 0U; i < rake; i++)
        // {
        //     float inter_distance = 0.F;
        //     for (auto j = 0U; j < Robot::dimension; j++)
        //     {
        //         if (i == 0)
        //         {
        //             diff_arr[i + j * rake] = initial_projected_block[{j, i}] - start_block[{j, i}];
        //         }
        //         else
        //         {
        //             diff_arr[i + j * rake] = initial_projected_block[{j, i}] - initial_projected_block[{j, i-1}];
        //         }
        //         inter_distance = inter_distance + diff_arr[i + j * rake] * diff_arr[i + j * rake];
        //     }
        //     if (inter_distance > 4 * (distance / rake) * (distance / rake))
        //     {
        //         invalid_distance_counter_outside++;
        //         // std::cout << "Invalid config due to distance constraint" << inter_distance << " at " << i << " with max distance allowed is " << 4 * (distance / rake) * (distance / rake) << std::endl;
        //         return false;
        //     }
        //     max_inter_dist = std::max(max_inter_dist, inter_distance);
        // }
        // max_inter_dist = std::sqrt(max_inter_dist);
        // typename Robot::template ConfigurationBlock<rake> shifted_block = typename Robot::template ConfigurationBlock<rake>(diff_arr);

        // std::cout << "Attachment: " << environment.eef_attachments.size() << std::endl;
        bool valid = (environment.eef_attachments.size()) ?
                         Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                         Robot::template fkcc<rake>(environment, initial_projected_block);

        typename Robot::ConfigurationArray last_projected;
        for (auto i = rake-1; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = initial_projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }
        // std::cout << initial_projected_block << std::endl;
        auto max_inter_dist = std::sqrt(inter_rake_distance.hmax());

        if (not valid or max_inter_dist <= (distance / rake))
        {
            if (not valid)
                collision_counter++;
            return valid;
        }

        n = std::max(std::ceil(max_inter_dist * resolution), 1.F);

        for (auto i = 1U; i < n; ++i)
        {

            initial_projected_block = initial_projected_block - shifted_block / n;
            if (not constraint.projectConfiguration(initial_projected_block, projected_block, projection_method, max_inter_dist * rake, projection_descent_rate))
            {
                unable_to_project_inside_counter++;
                return false;
            }
            auto q_dist = (projected_block[0] - initial_projected_block[0]) * (projected_block[0] - initial_projected_block[0]);
            for(auto j = 1U; j < Robot::dimension; j++)
                q_dist = q_dist + (projected_block[j] - initial_projected_block[j]) * (projected_block[j] - initial_projected_block[j]);
            if (q_dist.test_any_greater(4 * max_inter_dist / n * max_inter_dist / n))
            {
                invalid_distance_counter_inside++;
                // std::cout << q_dist << " " << q_dist.sqrt() << " " << max_inter_dist << std::endl;
                // std::cout << projected_block << "," << initial_projected_block << "," << shifted_block / n <<std::endl;
                return false;
            }


            bool valid_inside = (environment.eef_attachments.size()) ?
                             Robot::template fkcc_attach<rake>(environment, projected_block) :
                             Robot::template fkcc<rake>(environment, projected_block);

            if (not valid_inside)
            {
                collision_inside_counter++;
                // std::cout << "Collision Detected Inside" << std::endl;
                return false;
            }

            // for (auto r = 0U; r < rake; r++)
            // {
            //     for (auto j = 0U; j < Robot::dimension; j++)
            //     {
            //         last_projected[j] = projected_block[{j, r}];
            //     }
            //     projected_vector.push_back(typename Robot::Configuration(last_projected));
            // }
        }
        // std::cout << "projected : " <<  typename Robot::Configuration(last_projected) << "from " << start << "block " <<  block << " proj block " <<  projected_block << " to " <<  start + vector << n << std::endl;
        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline constexpr auto project_constraint_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<typename Robot::Configuration> &projected_vector,
        ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const collision::Environment<FloatVector<rake>> &environment,
        ProjMethod projection_method = ProjMethod::InnerLM,
        float projection_descent_rate = 1.0) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(
            start, vector, vector.l2_norm(), projected_vector, constraint, environment, projection_method, projection_descent_rate);
    }
}  // namespace vamp::planning
