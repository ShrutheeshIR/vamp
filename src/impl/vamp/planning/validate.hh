#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/planning/nn.hh>
#include <vamp/collision/environment.hh>
#include <chrono>
#include <iomanip>

namespace vamp::planning
{
    using namespace std::chrono;
    using namespace std;
    struct RRTCProfiler {

        vector<double> step2_times;
        vector<double> step3_times;
        vector<double> step4_times;
        vector<double> step6_times;
        vector<double> step8_times;
        vector<double> step10_times;
        vector<double> step12_times;
        vector<double> step14_times;
        vector<double> step15_times;
        vector<double> step16_times;

        vector<double> setup_before_validate_times;
        vector<double> validate_constraint_vector_times;
        vector<double> insert_projected_to_tree_times;
        vector<double> validate_extend_constraint_vector_times;
        vector<double> insertt_extended_to_tree_times;
        vector<double> full_extend_times;
        vector<double> one_iteration_times;
        //vector<double> distance_function_times;
        //vector<double> copy_function_times;
        //vector<double> project_step_times;
        //vector<double> full_project_times;
        //vector<double> copy_old_to_new_times;
        //vector<double> q_dist_times;
        void print_vector(vector<double> v) {
            for (double i: v)
                std::cout << i << ' ';
            std::cout << "\n";
        }

        void report(const string& stepName, const vector<double>& times) {
            if (times.empty()) return;

            double avg = accumulate(times.begin(), times.end(), 0.0) / times.size();

            vector<double> sorted = times;
            sort(sorted.begin(), sorted.end());
            double median = (sorted.size() % 2 == 0) 
                            ? (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]) / 2.0
                            : sorted[sorted.size()/2];

            cout << stepName << ": "
                << "avg = " << avg << " us, "
                << "median = " << median << " us, " << "sample size = " << sorted.size() <<"\n";
        }

        void printReport() {
            report("Initialize blocks", step2_times);
            report("Create block vec", step3_times);
            report("compute n", step4_times);
            report("fk check", step6_times);
            report("return valid", step8_times);
            report("Compute backstep", step10_times);
            report("Compute new block", step12_times);
            report("FK inside", step14_times);
            report("Full loop", step15_times);
            report("Setup before validate", setup_before_validate_times);
            report("Validate vector ", validate_constraint_vector_times);
            report("Insert vector to tree", insert_projected_to_tree_times);
            report("Validate extension vector", validate_extend_constraint_vector_times);
            report("Insert extension tree", insertt_extended_to_tree_times);
            report("Full extent", full_extend_times);
            report("One iteration", one_iteration_times);
            print_vector(step6_times);
            std::cout << "thisis step6\n";
            print_vector(step14_times);
            std::cout << "thisis step14\n";
            //report("Distance Fn", distance_function_times);
            //report("Copy fn", copy_function_times);
            //report("One project cholesky step", project_step_times);
            //report("Copy old to new", copy_old_to_new_times);
            //report("Compute q dist", q_dist_times);
            //report("Full project", full_project_times);

        }
    };
    inline RRTCProfiler profiler_rrtc;

    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        auto t0 = high_resolution_clock::now();
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;
        auto t2 = high_resolution_clock::now();
        profiler_rrtc.step2_times.push_back(duration<double, micro>(t2 - t0).count());
        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }
        auto t3 = high_resolution_clock::now();
        profiler_rrtc.step3_times.push_back(duration<double, micro>(t3 - t2).count());

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);
        auto t4 = high_resolution_clock::now();
        profiler_rrtc.step4_times.push_back(duration<double, micro>(t4 - t3).count());

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        auto t6 = high_resolution_clock::now();
        profiler_rrtc.step6_times.push_back(duration<double, micro>(t6 - t4).count());
        if (not valid or n == 1)
        {
            return valid;
        }
        auto t8 = high_resolution_clock::now();
        profiler_rrtc.step8_times.push_back(duration<double, micro>(t8 - t6).count());
        const auto backstep = vector / (rake * n);
        auto t10 = high_resolution_clock::now();
        profiler_rrtc.step10_times.push_back(duration<double, micro>(t10 - t8).count());
        std::cout << "inside of validate.hh, n is " << n << "\n";
        std::cout << "to calculate n, distance of " << std::setprecision(15) << distance << " was used, rake of " << std::setprecision(15) << static_cast<float>(rake) << " was used \n";
        std::cout << "result was this before ceiling operation:" << std::setprecision(15) << (distance / static_cast<float>(rake) * resolution) << "\n";
        for (auto i = 1U; i < n; ++i)
        {
            auto t11 = high_resolution_clock::now();
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }
            auto t12 = high_resolution_clock::now();
            profiler_rrtc.step12_times.push_back(duration<double, micro>(t12 - t11).count());
            if (not Robot::template fkcc<rake>(environment, block))
            {
                return false;
            }
            auto t14 = high_resolution_clock::now();
            profiler_rrtc.step14_times.push_back(duration<double, micro>(t14 - t12).count());
        }
        auto t15 = high_resolution_clock::now();
        profiler_rrtc.step15_times.push_back(duration<double, micro>(t15 - t10).count());
        profiler_rrtc.step16_times.push_back(duration<double, micro>(t15 - t0).count());
        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
    }
}  // namespace vamp::planning
