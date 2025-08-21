#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraints.hh>

namespace vamp::planning
{
    // template <std::size_t n, std::size_t... I>
    // inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    // {
    //     return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    // }

    // template <std::size_t n>
    // struct Percents
    // {
    //     inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    // };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto project_constraint_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        std::vector<typename Robot::Configuration> &projected_vector, 
        TaskSpaceConstraint<Robot> &constraint,
        const collision::Environment<FloatVector<rake>> &environment
        ) -> bool
    {

        projected_vector.clear();
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        // const auto percents = FloatVector<rake>(Percents<rake>::percents);
        typename Robot::template ConfigurationArray block;
        bool project_success = false;
        
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.to_array()[i];

        }


        // typename Robot::template ConfigurationBlock<rake> block;

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.to_array()[j];
            }
            typename Robot::template ConfigurationArray projected_config;
            project_success = constraint.project(block, projected_config);
            if (project_success == false)
            {
                return false;
            }
            if (not Robot::template fkcc<rake>(environment, typename Robot::Configuration(block)))
            {
                return false;
            }

            projected_vector.push_back(typename Robot::Configuration(projected_config));
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto project_constraint_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<typename Robot::Configuration> &projected_vector, 
        TaskSpaceConstraint<Robot> &constraint,
        const collision::Environment<FloatVector<rake>> &environment
        ) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), projected_vector, constraint);
    }
}  // namespace vamp::planning
