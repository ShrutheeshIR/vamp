#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
#include <vamp/vector/math.hh>
#include <iomanip>
namespace vamp::planning::constraint
{


template <typename Robot, std::size_t rake, typename... Constraints>
    class ComposableConstraints : public RobotConstraint<Robot, rake, ComposableConstraints<Robot, rake, Constraints...>>
    {
        std::tuple<Constraints...> constraints_;

    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        ConfigurationBlock q_old;

    public:
        using ConstraintPack = std::tuple<Constraints...>;
        static constexpr std::size_t total_size = (Constraints::size + ...);

        explicit ComposableConstraints(Constraints... cs)
            : constraints_(std::move(cs)...) {}

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const {
            return std::apply([&](const auto&... c) {
                return (c.distanceToConstraint(q) + ...);
            }, constraints_);
        }

        void print_robot_tsr_error(const ConfigurationBlock &q) const {
            std::apply([&](const auto&... c) { (c.print_robot_tsr_error(q), ...); }, constraints_);
        }

        vamp::FloatVector<rake, 1> projectStep(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0f)
        {
            ConfigurationBlock q_in = q;

            std::apply([&](auto&... c) {
                ((c.projectStep(q_in, q_new, projection_method, alpha), q_in = q_new), ...);
            }, constraints_);

            return distanceToConstraint(q_new);
        }
        // vamp::FloatVector<rake, 1> projectStep(
        //     const ConfigurationBlock &q,
        //     ConfigurationBlock &q_new,
        //     ProjMethod projection_method = ProjMethod::InnerLM,
        //     float alpha = 1.0f)
        // {
        //     ConfigurationBlock q_in = q;

        //     std::apply([&](auto&... c) {
        //         ((
        //             // Print BEFORE calling projectStep
        //             std::cout << "Before projectStep for constraint: "
        //                       << c.name << std::endl,

        //             // Call projectStep
        //             c.projectStep(q_in, q_new, projection_method, alpha),

        //             // Update q_in
        //             q_in = q_new,

        //             // Print AFTER calling projectStep
        //             std::cout << "After projectStep for constraint: "
        //                       << c.name << std::endl
        //         ), ...);  // fold over all constraints
        //     }, constraints_);

        //     return distanceToConstraint(q_new);
        // }

            bool projectConfiguration(
                const ConfigurationBlock &q,
                ConfigurationBlock &q_new,
                ProjMethod projection_method = ProjMethod::InnerLM,
                float max_q_dist = 5.0F,
                float descend_rate = 1.0F,
                int num_projection_iterations = 25,
                bool verbose = false)
            {
                /**
                * project a configuration block in parallel onto the constraint manifold
                * @param q - original config
                * @param q_new - projected config
                * @param projection_method - something from ProjMethod
                * @param max_q_dist - break out early if projected config is farther than max_q_dist away from
                * start
                *
                * @return success of projection
                */

                bool success = false;
                auto dist = distanceToConstraint(q);

                size_t project_iter = 0;
                q_new = q;
                q_old = q;

                // std::cout << q << std::endl;

                while ((project_iter < num_projection_iterations) and (not dist.test_all_less_equal(0.0001F)))
                {
                    dist = projectStep(q_old, q_new, projection_method, descend_rate);
                    // std::cout << "Iteration " << project_iter << " Distance: " << dist << std::endl;
                    // std::cout << q_old << q_new << std::endl;
                    auto q_dist_from_prev = (q_new[0] - q_old[0]) * (q_new[0] - q_old[0]);
                    auto q_dist_from_start = (q_new[0] - q[0]) * (q_new[0] - q[0]);

                    for (auto i = 1U; i < Robot::dimension; i++)
                    {
                        q_dist_from_prev = q_dist_from_prev + (q_new[i] - q_old[i]) * (q_new[i] - q_old[i]);
                        q_dist_from_start = q_dist_from_start + (q_new[i] - q[i]) * (q_new[i] - q[i]);
                    }

                    // std::cout << q_dist_from_prev << " " << dist << std::endl;
                    if (q_dist_from_prev.test_all_less_equal(0.000001F))  // if i make no forward progress
                    {
                        // std::cout << "Minimal progress " << dist << q_dist_from_prev << std::endl << q << std::endl;
                        break;
                    }

                    if (q_dist_from_prev.test_any_greater(4 * max_q_dist * max_q_dist))  // from triangle
                                                                                                // inequality
                    {
                        // std::cout << "Too large step " << q_dist_from_prev << std::endl;
                        // std::cout << q_old << std::endl;
                        break;
                    }
                    q_old = q_new;
                    project_iter += 1;
                }
                if (dist.test_all_less_equal(0.0001F))
                {
                    success = true;
                }
                if (verbose)
                {
                    std::cout << "Num projection steps : " << project_iter << " "<< dist << " and success : " << success << " " << std::endl;
                    std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;
                }

                return success;
            }

            int projectAnyConfiguration(
                const ConfigurationBlock &q,
                ConfigurationBlock &q_new,
                ProjMethod projection_method = ProjMethod::InnerLM,
                float max_q_dist = 5.0F,
                float descend_rate = 1.0F,
                int num_projection_iterations = 25,
                bool verbose = false)
            {
                /**
                * project a configuration block in parallel onto the constraint manifold
                * @param q - original config
                * @param q_new - projected config
                * @param projection_method - something from ProjMethod
                * @param max_q_dist - break out early if projected config is farther than max_q_dist away from
                * start
                *
                * @return the rake position of the successfully projected configuration, -1 if all failed
                *         priority is implicitly encoded in the rake position (lower is higher priority)
                */

                int success_position = -1;
                auto dist = distanceToConstraint(q);

                size_t project_iter = 0;
                q_new = q;
                q_old = q;

                // std::cout << q << std::endl;

                while ((project_iter < num_projection_iterations) and (not dist.test_any_less_equal(0.0001F)))
                {
                    dist = projectStep(q_old, q_new, projection_method, descend_rate);
                    // std::cout << "Iteration " << project_iter << " Distance: " << dist << std::endl;
                    // std::cout << q_old << q_new << std::endl;
                    auto q_dist_from_prev = (q_new[0] - q_old[0]) * (q_new[0] - q_old[0]);
                    auto q_dist_from_start = (q_new[0] - q[0]) * (q_new[0] - q[0]);

                    for (auto i = 1U; i < Robot::dimension; i++)
                    {
                        q_dist_from_prev = q_dist_from_prev + (q_new[i] - q_old[i]) * (q_new[i] - q_old[i]);
                        q_dist_from_start = q_dist_from_start + (q_new[i] - q[i]) * (q_new[i] - q[i]);
                    }

                    // std::cout << q_dist_from_prev << " " << dist << std::endl;
                    if (q_dist_from_prev.test_all_less_equal(0.000001F))  // if i make no forward progress in any of them
                    {
                        // std::cout << "Minimal progress " << dist << q_dist_from_prev << std::endl << q << std::endl;
                        break;
                    }

                    if (q_dist_from_prev.test_all_greater_equal(4 * max_q_dist * max_q_dist + 1e-6F))  // from triangle
                                                                                                // inequality
                    {
                        // std::cout << "Too large step " << q_dist_from_prev << std::endl;
                        // std::cout << q_old << std::endl;
                        break;
                    }
                    q_old = q_new;
                    project_iter += 1;
                }
                if (dist.test_any_less_equal(0.0001F))
                {
                    for(size_t i = 0; i < rake; i++){
                        if (dist[{0, i}] <= 0.0001F){
                            success_position = i;
                            break;
                        }
                    }
                }
                if (verbose)
                {
                    std::cout << "Num projection steps : " << project_iter << " "<< dist << " and success : " << success_position << " " << std::endl;
                    std::cout << "Num steps : " << project_iter << " and success : " << success_position << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;
                }

                return success_position;
            }


    };
}  // namespace vamp::planning::constraint