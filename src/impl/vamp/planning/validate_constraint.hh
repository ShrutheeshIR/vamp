#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate.hh>

namespace vamp::planning
{

    template <typename Robot, std::size_t rake, std::size_t resolution, typename... Constraints>
    inline constexpr auto project_constraint_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        std::vector<typename Robot::Configuration> &projected_vector,
        ComposableConstraints<Robot, rake, Constraints...> &constraint,
        const collision::Environment<FloatVector<rake>> &environment,
        ProjMethod projection_method = ProjMethod::GradDesc,
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
            return ableToProject;
        }
        float max_inter_dist = 0.F;

        // Compute inter-rake differences
        std::array<vamp::FloatT, Robot::dimension * rake> diff_arr;
        for (auto i = 0U; i < rake; i++)
        {
            float inter_distance = 0.F;
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                if (i == 0)
                {
                    diff_arr[i + j * rake] = initial_projected_block[{j, i}] - start.broadcast(j)[{j, 0}];
                }
                else
                {
                    diff_arr[i + j * rake] = initial_projected_block[{j, i}] - initial_projected_block[{j, i-1}];
                }
                inter_distance = inter_distance + diff_arr[i + j * rake] * diff_arr[i + j * rake];
            }
            if (inter_distance > 4 * (distance / rake) * (distance / rake))
            {
                // std::cout << "Invalid config due to distance constraint" << std::endl;
                return false;
            }
            max_inter_dist = std::max(max_inter_dist, inter_distance);
        }
        max_inter_dist = std::sqrt(max_inter_dist);
        typename Robot::template ConfigurationBlock<rake> shifted_block = typename Robot::template ConfigurationBlock<rake>(diff_arr);


        bool valid = (environment.eef_attachments.size()) ?
                         Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                         Robot::template fkcc<rake>(environment, initial_projected_block);

        typename Robot::ConfigurationArray last_projected;
        for (auto i = 0U; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = initial_projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }
        // std::cout << initial_projected_block << std::endl;

        if (not valid or max_inter_dist < (distance / rake))
        {
            // std::cout << "Invalid config " << valid << std::endl;
            return valid;
        }


        n = std::max(std::ceil(max_inter_dist * resolution), 1.F);

        for (auto i = 1U; i < n; ++i)
        {

            initial_projected_block = initial_projected_block - shifted_block / n;
            if (not constraint.projectConfiguration(initial_projected_block, projected_block, projection_method, max_inter_dist * rake, projection_descent_rate))
            {
                return false;
            }
            auto q_dist = (projected_block[0] - initial_projected_block[0]) * (projected_block[0] - initial_projected_block[0]);
            for(auto j = 1U; j < Robot::dimension; j++)
                q_dist = q_dist + (projected_block[j] - initial_projected_block[j]) * (projected_block[j] - initial_projected_block[j]);
            if (q_dist.test_any_greater_equal(4 * max_inter_dist / n * max_inter_dist / n))
            {
                std::cout << q_dist << " " << q_dist.sqrt() << " " << max_inter_dist << std::endl;
                std::cout << projected_block << "," << initial_projected_block << "," << shifted_block / n <<std::endl;
                return false;
            }



            if (not Robot::template fkcc<rake>(environment, projected_block))
            {
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
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(
            start, vector, vector.l2_norm(), projected_vector, constraint, environment);
    }
}  // namespace vamp::planning
