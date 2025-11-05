#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
#include <vamp/vector/math.hh>

namespace vamp::planning
{

    // exposing to public so we can set this in settings maybe?
    enum ProjMethod
    {
        InnerLM,  // J.T * ((J.JT + l)-1 . err) --> (6,6) matrix inv
        OuterLM,  // (JT.T + l)-1 . (J.T * err) -> (nq,nq) matrix inv
        GradDesc  // J.T * err
    };

    // This is an abstract class
    template <typename Robot, std::size_t rake>
    class RobotConstraint
    {
    protected:
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        template <std::size_t dim>
        inline static auto assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
        {
            for (size_t i = 0; i < dim; i++)
            {
                dest[i] = src[i];
            }
        }


    public:
        // virtual vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) = 0;

        void integrateJointConfiguration(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            const ConfigurationBlock &gradient,
            float alpha = 1.0)
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                q_new[i] = q[i] - gradient[i] * alpha;
            }
            Robot::descale_configuration_block(q_new);
            q_new = q_new.clamp(0.F, 1.F);
            Robot::scale_configuration_block(q_new);
        }

        // virtual vamp::FloatVector<rake, 1> projectStep(
        //     const ConfigurationBlock &q,
        //     ConfigurationBlock &q_new,
        //     ProjMethod projection_method = ProjMethod::InnerLM,
        //     bool update_q = true) = 0;

    };

    template <typename Robot, std::size_t rake>
    class TaskSpaceConstraint : public RobotConstraint<Robot, rake>
    {
        /**
         * A TSR constraint is expressed as 2 transformation matrices
         * with a lower and upper bound, as per
         * https://personalrobotics.cs.washington.edu/publications/berenson2011task.pdf
         *
         *
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        std::pair<std::array<float, 6>, std::array<float, 6>> bounds;  // error bounds in se3

        Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference;
        Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world;

        struct TSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7 * Robot::n_eef> rTeB;
            vamp::FloatVector<rake, 7 * Robot::n_eef> wTrB;
            vamp::FloatVector<rake, 6 * Robot::n_eef> lbB;
            vamp::FloatVector<rake, 6 * Robot::n_eef> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t eef_id = (index - Robot::dimension) / size_per_eef;
                size_t index_e = (index - Robot::dimension) % size_per_eef;

                if (index_e < 7)  // rtE
                {
                    return rTeB[eef_id * 7 + index_e];
                }

                else if (index_e >= 7 && index_e < (2 * 7))
                {
                    return wTrB[eef_id * 7 + index_e - 7];
                }

                else if (index_e >= (2 * 7) && index_e < (2 * 7 + 6))
                {
                    return lbB[eef_id * 6 + index_e - (2 * 7)];
                }

                else if (index_e >= (2 * 7 + 6) && index_e < (2 * 7 + 6 * 2))
                {
                    return ubB[eef_id * 6 + index_e - (2 * 7 + 6)];
                }
                else
                    return q[0];
            }
        };

        TSRComputeInput tsr_function_inp;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 6 * Robot::n_eef * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 6 * Robot::n_eef> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 6 * Robot::n_eef * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * Robot::n_eef * Robot::dimension &&
                    index < 6 * Robot::n_eef * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - 6 * Robot::n_eef * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 6 * Robot::n_eef * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * Robot::n_eef * Robot::dimension &&
                    index < 6 * Robot::n_eef * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - 6 * Robot::n_eef * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, 6 * Robot::n_eef + 6 * Robot::n_eef * Robot::dimension> y)
            {
                for (size_t i = 0; i < 6 * Robot::n_eef; i++)
                {
                    err[i] = y[6 * Robot::n_eef * Robot::dimension + i];
                }
                for (size_t i = 0; i < 6 * Robot::n_eef * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        JacobianProjectInp jac_proj_inp;
        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        TaskSpaceConstraint(
            const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference,  // rTe
            const Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world,         // wTr
            const std::pair<std::array<float, 6>, std::array<float, 6>> bounds)
          : eef_pose_w_ref_reference(eef_pose_w_ref_reference)
          , ref_frame_w_world(ref_frame_w_world)
          , bounds(bounds)
        {
            Eigen::Quaternion<float> q1(eef_pose_w_ref_reference.linear());
            std::array<float, 7> transform1 = {
                q1.w(),
                q1.x(),
                q1.y(),
                q1.z(),
                eef_pose_w_ref_reference.translation().x(),
                eef_pose_w_ref_reference.translation().y(),
                eef_pose_w_ref_reference.translation().z()};

            Eigen::Quaternion<float> q2(ref_frame_w_world.linear());
            std::array<float, 7> transform2 = {
                q2.w(),
                q2.x(),
                q2.y(),
                q2.z(),
                ref_frame_w_world.translation().x(),
                ref_frame_w_world.translation().y(),
                ref_frame_w_world.translation().z()};
            RobotConstraint<Robot, rake>::template assignBlock<7>(transform1, tsr_function_inp.rTeB);
            RobotConstraint<Robot, rake>::template assignBlock<7>(transform2, tsr_function_inp.wTrB);

            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.first, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.second, tsr_function_inp.ubB);
        }

        auto print_robot_tsr_error(const ConfigurationBlock &q)
        {
            auto dist = distanceToConstraint(q);
            // for(auto i=0U; i < 6 * Robot::n_eef * Robot::dimension; i++)
            //     std::cout << jac_proj_inp.J[{i, 0}] << " ";
            // std::cout << std::endl;
            for(auto i=0U; i < 6 * Robot::n_eef; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q)
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            Robot::template tsr_error<rake>(tsr_function_inp, jac_proj_inp);

            const size_t jac_offset = 6 * Robot::n_eef * Robot::dimension;
            for (size_t i = 0; i < 6 * Robot::n_eef; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            }
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6 * Robot::n_eef; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }

            return d;
        }

        vamp::FloatVector<rake, 1> projectStep(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            bool update_q = true,
            float alpha = 1.0)
        {
            auto dist = distanceToConstraint(q);
            if (update_q)
            {
                ConfigurationBlock grad;

                if (projection_method == ProjMethod::InnerLM)
                {
                    Robot::template solve_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
                }
                RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            }
            return dist;
        }
        bool projectConfiguration(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float max_q_dist = 5.0,
            float descend_rate = 1.0)
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
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                q_new[i] = q[i];
                q_old[i] = q[i];
            }

            while ((project_iter < 100) and (not dist.test_all_less_equal(0.0001F)))
            {
                dist = projectStep(q_old, q_new, projection_method, true, descend_rate);
                auto q_dist_from_prev = (q_new[0] - q_old[0]) * (q_new[0] - q_old[0]);
                auto q_dist_from_start = (q_new[0] - q[0]) * (q_new[0] - q[0]);

                for (auto i = 1U; i < Robot::dimension; i++)
                {
                    q_dist_from_prev = q_dist_from_prev + (q_new[i] - q_old[i]) * (q_new[i] - q_old[i]);
                    q_dist_from_start = q_dist_from_start + (q_new[i] - q[i]) * (q_new[i] - q[i]);
                }

                if (q_dist_from_prev.test_all_less_equal(0.00001F))  // if i make no forward progress
                {
                    break;
                }

                // if (q_dist_from_prev.test_any_greater_equal(4 * max_q_dist * max_q_dist))  // from triangle
                //                                                                             // inequality
                // {
                //     break;
                // }
                q_old = q_new + 0.0;
                project_iter += 1;
            }
            if (dist.test_all_less_equal(0.0001F))
            {
                success = true;
            }
            // std::cout << "Num projection steps : " << project_iter << " ";
            // std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;

            return success;
        }


    };

}  // namespace vamp::planning