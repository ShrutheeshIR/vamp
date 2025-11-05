#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraint.hh>
#include <vamp/planning/validate.hh>

namespace vamp::planning
{

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto project_constraint_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        std::vector<typename Robot::Configuration> &projected_vector,
        TaskSpaceConstraint<Robot, rake> &constraint,
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
            return ableToProject;
        }

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

        if (not valid or n == 1)
        {
            return valid;
        }

        const typename Robot::Configuration new_vector =
            typename Robot::Configuration(last_projected) - start;

        auto q_dist = new_vector.squared_l2_norm();
        if (q_dist > 2 * distance * distance)  // projected too far
        {
            return false;
        }

        n = std::max(std::ceil(q_dist / static_cast<float>(rake) * resolution), 1.F);
        // std::cout << " N : " << n << " qdist " << q_dist << " res " << resolution << " new vector " << new_vector << std::endl;
        const auto backstep = new_vector / (rake * n);

        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                initial_projected_block[j] = initial_projected_block[j] - backstep.broadcast(j);
            }

            if (not constraint.projectConfiguration(initial_projected_block, projected_block, projection_method, q_dist))
            {
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

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto project_constraint_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<typename Robot::Configuration> &projected_vector,
        TaskSpaceConstraint<Robot, rake> &constraint,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(
            start, vector, vector.l2_norm(), projected_vector, constraint, environment);
    }
}  // namespace vamp::planning
