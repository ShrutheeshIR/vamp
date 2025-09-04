#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/task_space_constraints.hh>
#include <vamp/planning/validate.hh>
#include <chrono>


using namespace std::chrono;
using namespace std;

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
        auto t0 = high_resolution_clock::now();
        projected_vector.clear();
        auto t1 = high_resolution_clock::now();
        profiler.step1_times.push_back(duration<double, micro>(t1 - t0).count());

        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;
        typename Robot::template ConfigurationBlock<rake> projected_block;
        typename Robot::template ConfigurationBlock<rake> initial_projected_block;

        auto t2 = high_resolution_clock::now();
        profiler.step2_times.push_back(duration<double, micro>(t2 - t1).count());
        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // std::cout << "--> Percents " << percents;
        for (auto i = 0U; i < Robot::dimension; ++i)
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);

        auto t3 = high_resolution_clock::now();
        profiler.step3_times.push_back(duration<double, micro>(t3 - t2).count());
        std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        auto t4 = high_resolution_clock::now();
        profiler.step4_times.push_back(duration<double, micro>(t4 - t3).count());

        bool ableToProject = constraint.project(block, initial_projected_block, distance);
        auto t5 = high_resolution_clock::now();
        if (not ableToProject)
            return ableToProject;
        profiler.step5_times.push_back(duration<double, micro>(t5 - t4).count());



        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, initial_projected_block) :
                                                 Robot::template fkcc<rake>(environment, initial_projected_block);
        auto t6 = high_resolution_clock::now();
        profiler.step6_times.push_back(duration<double, micro>(t6 - t5).count());

        typename Robot::ConfigurationArray last_projected;
        for (auto i = rake-1; i < rake; i++)
        {
            for (auto j = 0U; j < Robot::dimension; j++)
            {
                last_projected[j] = initial_projected_block[{j, i}];
            }
            projected_vector.push_back(typename Robot::Configuration(last_projected));
        }
        auto t7 = high_resolution_clock::now();
        profiler.step7_times.push_back(duration<double, micro>(t7 - t6).count());

        if (not valid or n == 1)
            return valid;



        auto t8 = high_resolution_clock::now();
        profiler.step8_times.push_back(duration<double, micro>(t8 - t7).count());
        const typename Robot::Configuration new_vector = typename Robot::Configuration(last_projected) - start;
        auto t9 = high_resolution_clock::now();
        profiler.step9_times.push_back(duration<double, micro>(t9 - t8).count());

        auto q_dist = new_vector.squared_l2_norm();
        if (q_dist > 2 * distance)
            return false;


        n = std::max(std::ceil(q_dist / static_cast<float>(rake) * resolution), 1.F);
        const auto backstep = new_vector / (rake * n);
        auto t10 = high_resolution_clock::now();
        profiler.step10_times.push_back(duration<double, micro>(t10 - t9).count());

        for (auto i = 1U; i < n; ++i)
        {
            auto t11 = high_resolution_clock::now();
            for (auto j = 0U; j < Robot::dimension; ++j)
                initial_projected_block[j] = initial_projected_block[j] - backstep.broadcast(j);

            auto t12 = high_resolution_clock::now();
            profiler.step12_times.push_back(duration<double, micro>(t12 - t11).count());
            if (not constraint.project(initial_projected_block, projected_block, q_dist))
                return false;

            auto t13 = high_resolution_clock::now();
            profiler.step13_times.push_back(duration<double, micro>(t13 - t12).count());
            if (not Robot::template fkcc<rake>(environment, projected_block))
                return false;
            auto t14 = high_resolution_clock::now();


            profiler.step14_times.push_back(duration<double, micro>(t14 - t13).count());

            // for (auto r = 0U; r < rake; r++)
            // {
            //     for (auto j = 0U; j < Robot::dimension; j++)
            //     {
            //         last_projected[j] = projected_block[{j, r}];
            //     }
            //     projected_vector.push_back(typename Robot::Configuration(last_projected));
            // }

        }
        auto t15 = high_resolution_clock::now();

        profiler.step15_times.push_back(duration<double, micro>(t15 - t10).count());
        profiler.step16_times.push_back(duration<double, micro>(t15 - t0).count());
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
