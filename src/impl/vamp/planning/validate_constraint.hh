#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraints.hh>
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
        const collision::Environment<FloatVector<rake>> &environment
        ) -> bool
    {
        // std::cout << "Started a round of validation with " << start << vector << std::endl;

        projected_vector.clear();

        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;
        typename Robot::template ConfigurationBlock<rake> projected_block;
        typename Robot::template ConfigurationBlock<rake> initial_projected_block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // std::cout << "--> Percents " << percents;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);

        std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);


        bool ableToProject = constraint.project(block, initial_projected_block, distance);
        if (not ableToProject)
            return ableToProject;



        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                                                 Robot::template fkcc<rake>(environment, initial_projected_block);

        typename Robot::ConfigurationArray last_projected;
        for (auto i = 0; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = initial_projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }

        if (not valid or n == 1)
            return valid;



        const typename Robot::Configuration new_vector = typename Robot::Configuration(last_projected) - start;

        auto q_dist = new_vector.squared_l2_norm();
        if (q_dist > 2 * distance)
            return false;


        n = std::max(std::ceil(q_dist / static_cast<float>(rake) * resolution), 1.F);
        const auto backstep = new_vector / (rake * n);

        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
                initial_projected_block[j] = initial_projected_block[j] - backstep.broadcast(j);

            if (not constraint.project(initial_projected_block, projected_block, q_dist))
                return false;

            if (not Robot::template fkcc<rake>(environment, projected_block))
                return false;

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
