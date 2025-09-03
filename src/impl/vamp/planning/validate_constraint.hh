#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraints.hh>
#include <vamp/planning/validate.hh>

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
        TaskSpaceConstraint<Robot, rake> &constraint,
        const collision::Environment<FloatVector<rake>> &environment
        ) -> bool
    {
        // std::cout << "Started a round of validation with " << start << vector << std::endl;

        projected_vector.clear();

        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;
        typename Robot::template ConfigurationBlock<rake> projected_block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // std::cout << "--> Percents " << percents;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);


        bool ableToProject = constraint.project(block, projected_block);
        if (not ableToProject)
        {
            // std::cout << "Unable to project initially : " << ableToProject<< std::endl;
            return ableToProject;
        }



        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, projected_block) :
                                                 Robot::template fkcc<rake>(environment, projected_block);

        // std::cout << "Block is " << block << std::endl;
        // std::cout << "Projected Block is " << projected_block << std::endl;

        typename Robot::ConfigurationArray last_projected;
        for (auto i = rake-1; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }

        if (not valid or n == 1)
        {
            // std::cout << "Unable to validate initially : " << valid << n << projected_block <<  block << std::endl;
            return valid;
        }
        // projected_vector.pop_back();



        const typename Robot::Configuration new_vector = typename Robot::Configuration(last_projected) - start;

        if (new_vector.squared_l2_norm() > 2 * distance)
            return false;
        // std::cout << "Validating connection vector " << new_vector << " from " << start << " to " << typename Robot::Configuration(last_projected) << std::endl;

        // extract out the last element from here, this is a 7 x 8 block, I need the 7 x 1 of the last element
        // auto new_vector = projected_block;

        const auto backstep = new_vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            if (not constraint.project(block, projected_block))
            {
                // std::cout << "Unable to project inside : " << ableToProject;
                return false;
            }

            if (not Robot::template fkcc<rake>(environment, projected_block))
            {
                // std::cout << "Unable to validate inside : " << valid << std::endl;
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
        // projected_vector.push_back(typename Robot::Configuration(last_projected));

        // for (auto i = 0U; i < Robot::dimension; i++) {  
        //     last_projected[i] = projected_block[{i, rake-1}];
        // }


        // std::cout << "projected : " <<  typename Robot::Configuration(last_projected) << "from " << start << block << projected_block << start + vector << std::endl;

        // projected_vector.push_back(typename Robot::Configuration(last_projected));
        return true;


    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto project_constraint_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        std::vector<typename Robot::Configuration> &projected_vector, 
        TaskSpaceConstraint<Robot, rake> &constraint,
        const collision::Environment<FloatVector<rake>> &environment
        ) -> bool
    {
        auto vector = goal - start;
        return project_constraint_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), projected_vector, constraint);
    }
}  // namespace vamp::planning
