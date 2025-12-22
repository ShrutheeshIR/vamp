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


    template <typename Robot, std::size_t rake, std::size_t num_polygons>
    class BimanualCoMTSRTaskSpaceConstraint : public RobotConstraint<Robot, rake>
    {
        /**
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        struct CoMConstraintInp
        {
            vamp::FloatVector<rake, 3> CoM;
            vamp::FloatVector<rake, 3 * Robot::dimension> com_jacobian;

            vamp::FloatVector<rake, 2 * num_polygons> polygon_points; // arranged as (x1, y1, x2, y2, x3, y3...)

            auto &operator[](size_t index)
            {

                if (index < 3)
                    return CoM[index];

                if (index >=3 && index < 3 + 3 * Robot::dimension)  // jacobian
                    return com_jacobian[index - 3];

                if (index >=3 + 3 * Robot::dimension)  // jacobian
                    return polygon_points[index - (3 + 3 * Robot::dimension)];

                else
                    return CoM[0];
            }

            const auto operator[](size_t index) const
            {
                if (index < 3)
                    return CoM[index];

                if (index >=3 && index < 3 + 3 * Robot::dimension)  // jacobian
                    return com_jacobian[index - 3];

                if (index >=3 + 3 * Robot::dimension)  // jacobian
                    return polygon_points[index - (3 + 3 * Robot::dimension)];

                else
                    return CoM[0];
            }

            CoMConstraintInp &
            operator=(vamp::FloatVector<rake, 3 + 3 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 3; i++)
                    CoM[i] = y[i];
                for (size_t i = 0; i < 3 * Robot::dimension; i++)
                    com_jacobian[i] = y[i + 3];

                for (size_t i = 0; i < 2 * num_polygons; i++)
                    polygon_points[i] = y[i + 3 + 3 * Robot::dimension];


                return *this;
            }
        };


        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, (2 + 6 + 6 * 2) * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, (2 + 6 + 6 * 2)> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < (2 + 6 + 6 * 2) * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= (2 + 6 + 6 * 2) *  Robot::dimension &&
                    index < (2 + 6 + 6 * 2) * Robot::dimension + (2 + 6 + 6 * 2))
                {
                    return err[index - (2 + 6 + 6 * 2) * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < (2 + 6 + 6 * 2) * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= (2 + 6 + 6 * 2) * Robot::dimension &&
                    index < (2 + 6 + 6 * 2) * Robot::dimension + (2 + 6 + 6 * 2))
                {
                    return err[index - (2 + 6 + 6 * 2) * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, (2 + 6 + 6 * 2) + (2 + 6 + 6 * 2) * Robot::dimension> y)
            {
                for (size_t i = 0; i < (2 + 6 + 6 * 2); i++)
                {
                    err[i] = y[(2 + 6 + 6 * 2) * Robot::dimension + i];
                }
                for (size_t i = 0; i < (2 + 6 + 6 * 2) * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        struct BimanualTSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7> rTlB;
            vamp::FloatVector<rake, 6> lbB;
            vamp::FloatVector<rake, 6> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t index_e = (index - Robot::dimension);

                if (index_e < 7)  // rTl
                {
                    return rTlB[index_e];
                }

                else if (index_e >= 7 && index_e < (1 * 7 + 6))
                {
                    return lbB[index_e - 7];
                }

                else if (index_e >= (1 * 7 + 6) && index_e < (1 * 7 + 6 * 2))
                {
                    return ubB[index_e - (7 + 6)];
                }
                else
                    return q[0];
            }
        };

        struct EEFPosTSRComputeInput
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

        EEFPosTSRComputeInput eef_pos_tsr_function_inp;
        BimanualTSRComputeInput tsr_function_inp;
        CoMConstraintInp com_jac_polygons;
        JacobianProjectInp jac_proj_inp;

        // size_t num_polygons;

        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        BimanualCoMTSRTaskSpaceConstraint(
            const std::array<float, 2 * num_polygons> polygon_points,
            const Eigen::Transform<float, 3, Eigen::Isometry> right_eef_pose_w_ref_left_eef,  // rTl
            const std::pair<std::array<float, 6>, std::array<float, 6>> bimanual_bounds,
            std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_pose_w_ref_reference,  // rTe
            std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> ref_frame_w_world,         // wTr
            const std::pair<std::array<float, 6 * Robot::n_eef>, std::array<float, 6 * Robot::n_eef>> eef_pos_bounds)
        {
            Eigen::Quaternion<float> q1(right_eef_pose_w_ref_left_eef.linear());
            std::array<float, 7> bimanual_transform = {
                q1.w(),
                q1.x(),
                q1.y(),
                q1.z(),
                right_eef_pose_w_ref_left_eef.translation().x(),
                right_eef_pose_w_ref_left_eef.translation().y(),
                right_eef_pose_w_ref_left_eef.translation().z()};

            RobotConstraint<Robot, rake>::template assignBlock<7>(bimanual_transform, tsr_function_inp.rTlB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bimanual_bounds.first, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bimanual_bounds.second, tsr_function_inp.ubB);

            for (size_t i = 0; i < 2 * num_polygons; i++)
                std::cout << polygon_points[i] << " ";
            std::cout << std::endl;


            RobotConstraint<Robot, rake>::template assignBlock<2 * num_polygons>(polygon_points, com_jac_polygons.polygon_points);

            // Now set up the individual TSR constraints
            std::array<float, 7 * Robot::n_eef> transform1;
            std::array<float, 7 * Robot::n_eef> transform2;

            for (auto eef_idx = 0U; eef_idx < Robot::n_eef; eef_idx++)
            {
                Eigen::Quaternion<float> q1(eef_pose_w_ref_reference[eef_idx].linear());
                transform1[7 * eef_idx + 0] = q1.w();
                transform1[7 * eef_idx + 1] = q1.x();
                transform1[7 * eef_idx + 2] = q1.y();
                transform1[7 * eef_idx + 3] = q1.z();
                transform1[7 * eef_idx + 4] = eef_pose_w_ref_reference[eef_idx].translation().x();
                transform1[7 * eef_idx + 5] = eef_pose_w_ref_reference[eef_idx].translation().y();
                transform1[7 * eef_idx + 6] = eef_pose_w_ref_reference[eef_idx].translation().z();

                Eigen::Quaternion<float> q2(ref_frame_w_world[eef_idx].linear());
                transform2[7 * eef_idx + 0] = q2.w();
                transform2[7 * eef_idx + 1] = q2.x();
                transform2[7 * eef_idx + 2] = q2.y();
                transform2[7 * eef_idx + 3] = q2.z();
                transform2[7 * eef_idx + 4] = ref_frame_w_world[eef_idx].translation().x();
                transform2[7 * eef_idx + 5] = ref_frame_w_world[eef_idx].translation().y();
                transform2[7 * eef_idx + 6] = ref_frame_w_world[eef_idx].translation().z();
            }

            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform1, eef_pos_tsr_function_inp.rTeB);
            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform2, eef_pos_tsr_function_inp.wTrB);
            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(eef_pos_bounds.first, eef_pos_tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(eef_pos_bounds.second, eef_pos_tsr_function_inp.ubB);


        }

        // auto print_robot_com_constraint_error(const ConfigurationBlock &q)
        // {
        //     // for(auto i=0U; i < Robot::dimension + 19; i++)
        //     //     std::cout << tsr_function_inp[i] << " ";


        //     auto dist = distanceToConstraint(q);
        //     for(auto i=0U; i < 6 * Robot::dimension; i++){
        //         if (i%Robot::dimension == 0)
        //             std::cout << std::endl;
        //         std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
        //     }
        //     std::cout << std::endl;
        //     for(auto i=0U; i < 6; i++)
        //         std::cout << jac_proj_inp.err[{i, 0}] << " ";
        //     std::cout << std::endl;

        // }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q)
        {


            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
                eef_pos_tsr_function_inp.q[i] = q[i];
            }

            vamp::FloatVector<rake, 6 * Robot::dimension + 6> tsr_err_jac;
            vamp::FloatVector<rake, 2 * Robot::dimension + 2> com_err_jac;
            vamp::FloatVector<rake, 6 * Robot::dimension * Robot::n_eef + 6 * Robot::n_eef> eef_pos_tsr_err_jac;


            Robot::template compute_com<rake>(q, com_jac_polygons);
            Robot::template com_constraint_error<rake>(com_jac_polygons, num_polygons, com_err_jac);
            Robot::template tsr_bimanual_error<rake>(tsr_function_inp, tsr_err_jac);
            Robot::template tsr_error<rake>(eef_pos_tsr_function_inp, eef_pos_tsr_err_jac);

            // feed the jac
            for (size_t i = 0; i < 2 * Robot::dimension; i++)
                jac_proj_inp[i] = com_err_jac[i];

            for (size_t i = 0; i < 6 * Robot::dimension; i++)
                jac_proj_inp[2 * Robot::dimension +i] = tsr_err_jac[i];

            for (size_t i = 0; i < 6 * 2 * Robot::dimension; i++)
                jac_proj_inp[8 * Robot::dimension +i] = eef_pos_tsr_err_jac[2 * 6 * Robot::dimension + i];

            // Copy the errors
            for (size_t i = 0; i < 2; i++)
                jac_proj_inp[i + (2 + 6 + 6 * 2) * Robot::dimension] = com_err_jac[2 * Robot::dimension + i];

            for (size_t i = 0; i < 6; i++)
                jac_proj_inp[i + (2 + 6 + 6 * 2) * Robot::dimension + 2] = tsr_err_jac[i + 6 * Robot::dimension];

            for (size_t i = 0; i < 6 * 2; i++)
                jac_proj_inp[i + (2 + 6 + 6 * 2) * Robot::dimension + 2 + 6] = eef_pos_tsr_err_jac[i + 6 * Robot::dimension * Robot::n_eef + 6 * 2];
            

            const size_t jac_offset = (2 + 6 + 6 * 2) * Robot::dimension + 2 + 6;
            for (size_t i = 0; i < 6 * 2; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - eef_pos_tsr_function_inp.lbB[6 * 2 + i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - eef_pos_tsr_function_inp.ubB[6 * 2 + i]).max(0.F);
            }


            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 2 + 6 + 6 * 2; i++)
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
                    Robot::template solve_bimanual_com_tsr_function_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_bimanual_com_tsr_function_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_bimanual_com_tsr_function_gradient_descent<rake>(jac_proj_inp, grad);
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

    template <typename Robot, std::size_t rake>
    class BimanualTaskSpaceConstraint : public RobotConstraint<Robot, rake>
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

        Eigen::Transform<float, 3, Eigen::Isometry> right_eef_pose_w_ref_left_eef;

        struct TSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7> rTlB;
            vamp::FloatVector<rake, 6> lbB;
            vamp::FloatVector<rake, 6> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t index_e = (index - Robot::dimension);

                if (index_e < 7)  // rTl
                {
                    return rTlB[index_e];
                }

                else if (index_e >= 7 && index_e < (1 * 7 + 6))
                {
                    return lbB[index_e - 7];
                }

                else if (index_e >= (1 * 7 + 6) && index_e < (1 * 7 + 6 * 2))
                {
                    return ubB[index_e - (7 + 6)];
                }
                else
                    return q[0];
            }
        };

        TSRComputeInput tsr_function_inp;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 6 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 6> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 *  Robot::dimension &&
                    index < 6 * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * Robot::dimension &&
                    index < 6 * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, 6 * Robot::n_eef + 6 * Robot::n_eef * Robot::dimension> y)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    err[i] = y[6 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 6 * Robot::dimension; i++)
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
        BimanualTaskSpaceConstraint(
            const Eigen::Transform<float, 3, Eigen::Isometry> right_eef_pose_w_ref_left_eef,  // rTl
            const std::pair<std::array<float, 6>, std::array<float, 6>> bounds)
          : right_eef_pose_w_ref_left_eef(right_eef_pose_w_ref_left_eef)
          , bounds(bounds)
        {
            Eigen::Quaternion<float> q1(right_eef_pose_w_ref_left_eef.linear());
            std::array<float, 7> transform1 = {
                q1.w(),
                q1.x(),
                q1.y(),
                q1.z(),
                right_eef_pose_w_ref_left_eef.translation().x(),
                right_eef_pose_w_ref_left_eef.translation().y(),
                right_eef_pose_w_ref_left_eef.translation().z()};

            RobotConstraint<Robot, rake>::template assignBlock<7>(transform1, tsr_function_inp.rTlB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.first, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.second, tsr_function_inp.ubB);
        }

        auto print_robot_tsr_error(const ConfigurationBlock &q)
        {
            // for(auto i=0U; i < Robot::dimension + 19; i++)
            //     std::cout << tsr_function_inp[i] << " ";


            auto dist = distanceToConstraint(q);
            for(auto i=0U; i < 6 * Robot::dimension; i++){
                if (i%Robot::dimension == 0)
                    std::cout << std::endl;
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            for(auto i=0U; i < 6; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q)
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            Robot::template tsr_bimanual_error<rake>(tsr_function_inp, jac_proj_inp);


            const size_t jac_offset = 6 * Robot::dimension;
            for (size_t i = 0; i < 6; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            }
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }
            // std::cout << "Error : ";
            // for(auto i=0U; i < 6; i++){
            //     std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            // }
            // std::cout << std::endl;

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
                    Robot::template solve_tsr_relative_error_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_tsr_relative_error_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_tsr_relative_error_gradient_descent<rake>(jac_proj_inp, grad);
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

                // if (q_dist_from_prev.test_all_less_equal(0.00001F))  // if i make no forward progress
                // {
                //     std::cout << "Minimal progress " << std::endl;
                //     break;
                // }

                if (q_dist_from_prev.test_any_greater_equal(4 * max_q_dist * max_q_dist))  // from triangle
                                                                                            // inequality
                {
                    break;
                }
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

    template <typename Robot, std::size_t rake, std::size_t num_polygons>
    class BimanualCoMTaskSpaceConstraint : public RobotConstraint<Robot, rake>
    {
        /**
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;
        std::pair<std::array<float, 6>, std::array<float, 6>> bounds;  // error bounds in se3
        Eigen::Transform<float, 3, Eigen::Isometry> right_eef_pose_w_ref_left_eef;


        struct CoMConstraintInp
        {
            vamp::FloatVector<rake, 3> CoM;
            vamp::FloatVector<rake, 3 * Robot::dimension> com_jacobian;

            vamp::FloatVector<rake, 2 * num_polygons> polygon_points; // arranged as (x1, y1, x2, y2, x3, y3...)

            auto &operator[](size_t index)
            {

                if (index < 3)
                    return CoM[index];

                if (index >=3 && index < 3 + 3 * Robot::dimension)  // jacobian
                    return com_jacobian[index - 3];

                if (index >=3 + 3 * Robot::dimension)  // jacobian
                    return polygon_points[index - (3 + 3 * Robot::dimension)];

                else
                    return CoM[0];
            }

            const auto operator[](size_t index) const
            {
                if (index < 3)
                    return CoM[index];

                if (index >=3 && index < 3 + 3 * Robot::dimension)  // jacobian
                    return com_jacobian[index - 3];

                if (index >=3 + 3 * Robot::dimension)  // jacobian
                    return polygon_points[index - (3 + 3 * Robot::dimension)];

                else
                    return CoM[0];
            }

            CoMConstraintInp &
            operator=(vamp::FloatVector<rake, 3 + 3 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 3; i++)
                    CoM[i] = y[i];
                for (size_t i = 0; i < 3 * Robot::dimension; i++)
                    com_jacobian[i] = y[i + 3];

                for (size_t i = 0; i < 2 * num_polygons; i++)
                    polygon_points[i] = y[i + 3 + 3 * Robot::dimension];


                return *this;
            }
        };


        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 8 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 8> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 8 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 8 *  Robot::dimension &&
                    index < 8 * Robot::dimension + 8)
                {
                    return err[index - 8 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 8 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 8 * Robot::dimension &&
                    index < 8 * Robot::dimension + 8)
                {
                    return err[index - 8 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, 8 + 8 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 8; i++)
                {
                    err[i] = y[8 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 8 * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        struct TSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7> rTlB;
            vamp::FloatVector<rake, 6> lbB;
            vamp::FloatVector<rake, 6> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t index_e = (index - Robot::dimension);

                if (index_e < 7)  // rTl
                {
                    return rTlB[index_e];
                }

                else if (index_e >= 7 && index_e < (1 * 7 + 6))
                {
                    return lbB[index_e - 7];
                }

                else if (index_e >= (1 * 7 + 6) && index_e < (1 * 7 + 6 * 2))
                {
                    return ubB[index_e - (7 + 6)];
                }
                else
                    return q[0];
            }
        };

        TSRComputeInput tsr_function_inp;
        CoMConstraintInp com_jac_polygons;
        JacobianProjectInp jac_proj_inp;

        // size_t num_polygons;

        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        BimanualCoMTaskSpaceConstraint(
            const std::array<float, 2 * num_polygons> polygon_points,
            const Eigen::Transform<float, 3, Eigen::Isometry> right_eef_pose_w_ref_left_eef,  // rTl
            const std::pair<std::array<float, 6>, std::array<float, 6>> bounds)
          : right_eef_pose_w_ref_left_eef(right_eef_pose_w_ref_left_eef)
          , bounds(bounds)
        {
            Eigen::Quaternion<float> q1(right_eef_pose_w_ref_left_eef.linear());
            std::array<float, 7> transform1 = {
                q1.w(),
                q1.x(),
                q1.y(),
                q1.z(),
                right_eef_pose_w_ref_left_eef.translation().x(),
                right_eef_pose_w_ref_left_eef.translation().y(),
                right_eef_pose_w_ref_left_eef.translation().z()};

            RobotConstraint<Robot, rake>::template assignBlock<7>(transform1, tsr_function_inp.rTlB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.first, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(bounds.second, tsr_function_inp.ubB);

            for (size_t i = 0; i < 2 * num_polygons; i++)
                std::cout << polygon_points[i] << " ";
            std::cout << std::endl;


            RobotConstraint<Robot, rake>::template assignBlock<2 * num_polygons>(polygon_points, com_jac_polygons.polygon_points);
        }

        // auto print_robot_com_constraint_error(const ConfigurationBlock &q)
        // {
        //     // for(auto i=0U; i < Robot::dimension + 19; i++)
        //     //     std::cout << tsr_function_inp[i] << " ";


        //     auto dist = distanceToConstraint(q);
        //     for(auto i=0U; i < 6 * Robot::dimension; i++){
        //         if (i%Robot::dimension == 0)
        //             std::cout << std::endl;
        //         std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
        //     }
        //     std::cout << std::endl;
        //     for(auto i=0U; i < 6; i++)
        //         std::cout << jac_proj_inp.err[{i, 0}] << " ";
        //     std::cout << std::endl;

        // }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q)
        {


            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            // for (size_t i = 0; i < Robot::dimension; i++)
            // {
            //     tsr_function_inp.q[i] = q[i];
            // }
            // std::cout << "Polygon points are :";
            // for(auto i=0U; i < 2 * num_polygons; i++){
            //     std::cout << std::setprecision(5) << com_jac_polygons.polygon_points[{i, 0}] << " ";
            // }
            // std::cout << std::endl;

            Robot::template compute_com<rake>(q, com_jac_polygons);
            // std::cout << "COM is :";
            // for(auto i=0U; i < 3; i++){
            //     std::cout << std::setprecision(5) << com_jac_polygons.CoM[{i, 0}] << " ";
            // }
            // std::cout << std::endl;
            // std::cout << "COM Jac is :";
            // for(auto i=0U; i < 3 * Robot::dimension; i++){
            //     if (i%Robot::dimension == 0)
            //         std::cout << std::endl;
            //     std::cout << std::setprecision(5) << com_jac_polygons.com_jacobian[{i, 0}] << " ";
            // }


            // std::cout << std::endl;
            // std::cout << "Polygon points are :";
            // for(auto i=0U; i < 2 * num_polygons; i++){
            //     std::cout << std::setprecision(5) << com_jac_polygons.polygon_points[{i, 0}] << " ";
            // }
            // std::cout << std::endl;

            vamp::FloatVector<rake, 2 * Robot::dimension + 2> com_err_jac;
            Robot::template com_constraint_error<rake>(com_jac_polygons, num_polygons, com_err_jac);


            vamp::FloatVector<rake, 6 * Robot::dimension + 6> tsr_err_jac;
            Robot::template tsr_bimanual_error<rake>(tsr_function_inp, tsr_err_jac);

            // feed the jac
            for (size_t i = 0; i < 2 * Robot::dimension; i++)
                jac_proj_inp[i] = com_err_jac[i];

            for (size_t i = 0; i < 6 * Robot::dimension; i++)
                jac_proj_inp[2 * Robot::dimension +i] = tsr_err_jac[i];

            for (size_t i = 0; i < 2; i++)
                jac_proj_inp[i + 8 * Robot::dimension] = com_err_jac[2 * Robot::dimension + i];

            for (size_t i = 0; i < 6; i++)
                jac_proj_inp[i + 8 * Robot::dimension + 2] = tsr_err_jac[i + 6 * Robot::dimension];


            // const size_t jac_offset = 6 * Robot::dimension;
            // for (size_t i = 0; i < 6; i++)
            // {
            //     jac_proj_inp[i + jac_offset] =
            //         (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
            //         (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            // }
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 8; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }

            return d;


            // std::cout << "Error : ";
            // for(auto i=0U; i < 8; i++){
            //     std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            // }
            // std::cout << std::endl;
            // std::cout << "Error Jac : ";
            // for(auto i=0U; i < 2 * Robot::dimension; i++){
            //     if (i%Robot::dimension == 0)
            //         std::cout << std::endl;
            //     std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            // }
            // std::cout << std::endl;
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
                    Robot::template solve_bimanual_com_function_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_bimanual_com_function_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_bimanual_com_function_gradient_descent<rake>(jac_proj_inp, grad);
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

                // if (q_dist_from_prev.test_all_less_equal(0.00001F))  // if i make no forward progress
                // {
                //     break;
                // }

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


        struct ShortenedJacobianProjectInp
        {
            vamp::FloatVector<rake, 6 * 2 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 6 * 2> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 6 * 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * 2 * Robot::dimension &&
                    index < 6 * 2 * Robot::dimension + 6 * 2)
                {
                    return err[index - 6 * 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 6 * 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * 2 * Robot::dimension &&
                    index < 6 * 2 * Robot::dimension + 6 * 2)
                {
                    return err[index - 6 * 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            ShortenedJacobianProjectInp &
            operator=(vamp::FloatVector<rake, 6 * 2 + 6 * 2 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 6 * 2; i++)
                {
                    err[i] = y[6 * 2 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 6 * 2 * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        ShortenedJacobianProjectInp short_jac_proj_inp;


        ConfigurationBlock q_old;

    public:
        TaskSpaceConstraint(
            std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> eef_pose_w_ref_reference,  // rTe
            std::array<Eigen::Transform<float, 3, Eigen::Isometry>, Robot::n_eef> ref_frame_w_world,         // wTr
            const std::pair<std::array<float, 6 * Robot::n_eef>, std::array<float, 6 * Robot::n_eef>> eef_pos_bounds)
        {


            std::array<float, 7 * Robot::n_eef> transform1;
            std::array<float, 7 * Robot::n_eef> transform2;

            for (auto eef_idx = 0U; eef_idx < Robot::n_eef; eef_idx++)
            {
                Eigen::Quaternion<float> q1(eef_pose_w_ref_reference[eef_idx].linear());
                transform1[7 * eef_idx + 0] = q1.w();
                transform1[7 * eef_idx + 1] = q1.x();
                transform1[7 * eef_idx + 2] = q1.y();
                transform1[7 * eef_idx + 3] = q1.z();
                transform1[7 * eef_idx + 4] = eef_pose_w_ref_reference[eef_idx].translation().x();
                transform1[7 * eef_idx + 5] = eef_pose_w_ref_reference[eef_idx].translation().y();
                transform1[7 * eef_idx + 6] = eef_pose_w_ref_reference[eef_idx].translation().z();

                Eigen::Quaternion<float> q2(ref_frame_w_world[eef_idx].linear());
                transform2[7 * eef_idx + 0] = q2.w();
                transform2[7 * eef_idx + 1] = q2.x();
                transform2[7 * eef_idx + 2] = q2.y();
                transform2[7 * eef_idx + 3] = q2.z();
                transform2[7 * eef_idx + 4] = ref_frame_w_world[eef_idx].translation().x();
                transform2[7 * eef_idx + 5] = ref_frame_w_world[eef_idx].translation().y();
                transform2[7 * eef_idx + 6] = ref_frame_w_world[eef_idx].translation().z();
            }

            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform1, tsr_function_inp.rTeB);
            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform2, tsr_function_inp.wTrB);

            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(eef_pos_bounds.first, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(eef_pos_bounds.second, tsr_function_inp.ubB);


        }

        auto print_robot_tsr_error(const ConfigurationBlock &q)
        {
            auto dist = distanceToConstraint(q);
            for(auto i=0U; i < 6 * Robot::n_eef * Robot::dimension; i++){
                if(i % Robot::dimension == 0)
                    std::cout << std::endl << i / Robot::dimension << " : ";
                std::cout << jac_proj_inp.J[{i, 0}] << " ";
            }

            std::cout << std::endl;
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
            const size_t short_jac_offset = 6 * 2 * Robot::dimension;

            for (size_t i = 0; i < 6 * Robot::n_eef; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);

            }

            for (std::size_t eef_idx = 0; eef_idx < 2; ++eef_idx) { // eef_idx = 0 -> i=2, 1 -> i=3
                std::size_t i = eef_idx + 2;

                for (std::size_t se3_idx = 0; se3_idx < 6; ++se3_idx) {
                    for (std::size_t dim = 0; dim < Robot::dimension; ++dim) {


                        std::size_t idxB = (eef_idx * 6 + se3_idx) * Robot::dimension + dim;
                        std::size_t idxA = (i * 6 + se3_idx) * Robot::dimension + dim;

                        short_jac_proj_inp[idxB] = jac_proj_inp[idxA];   // SIMD element assignment
                    }
                }
            }

            for (std::size_t eef_idx = 0; eef_idx < 2; ++eef_idx) { // eef_idx = 0 -> i=2, 1 -> i=3
                std::size_t i = eef_idx + 2;

                for (std::size_t se3_idx = 0; se3_idx < 6; ++se3_idx) {
                        std::size_t idxB = (eef_idx * 6 + se3_idx);
                        std::size_t idxA = (i * 6 + se3_idx);

                        short_jac_proj_inp[idxB + short_jac_offset] = jac_proj_inp[idxA + jac_offset];   // SIMD element assignment
                    }
                }
    


            // for(int i = 0U; i < 6 * Robot::n_eef * Robot::dimension; i++){
            //     if(i % Robot::dimension == 0)
            //         std::cout << std::endl << i/Robot::dimension << " : ";
            //     std::cout << jac_proj_inp.J[{i, 0}] << ", ";
            // }
            // std::cout << std::endl;

            // for(int i = 0U; i < 6 * 2 * Robot::dimension; i++){
            //     if(i % Robot::dimension == 0)
            //         std::cout << std::endl << i/Robot::dimension << " : ";
            //     std::cout << short_jac_proj_inp.J[{i, 0}] << ", ";
            // }
            // std::cout << std::endl;


            // for(int i = 0U; i < 6 * Robot::n_eef; i++)
            //     std::cout << jac_proj_inp.err[{i, 0}] << ", ";
            // std::cout << std::endl;

            // for(int i = 0U; i < 6 * 2; i++)
            //     std::cout << short_jac_proj_inp.err[{i, 0}] << ", ";
            // std::cout << std::endl;


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
                    Robot::template solve_2_eef_tsr_error_lm_inner<rake>(short_jac_proj_inp, grad);
                    // Robot::template solve_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_2_eef_tsr_error_lm_outer<rake>(short_jac_proj_inp, grad);
                    // Robot::template solve_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_2_eef_tsr_error_gradient_descent<rake>(short_jac_proj_inp, grad);
                    // Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
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

                // if (q_dist_from_prev.test_all_less_equal(0.00001F))  // if i make no forward progress
                // {
                //     break;
                // }

                // if (q_dist_from_prev.test_any_greater_equal(4 * max_q_dist * max_q_dist))  // from triangle
                //                                                                             // inequality
                // {
                //     break;
                // }
                q_old = q_new + 0.0;
                project_iter += 1;

                // for(auto i=0U; i < Robot::dimension; i++)
                //     std::cout << q_new[{i, 0}] << ",";
                // std::cout << std::endl;

            }
            if (dist.test_all_less_equal(0.0001F))
            {
                success = true;
            }
            std::cout << "Num projection steps : " << project_iter << " ";
            // std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;

            return success;
        }


    };

    template <typename Robot, std::size_t rake>
    class SETaskSpaceConstraint : public RobotConstraint<Robot, rake>
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

        struct TSRComputeInput
        {
            ConfigurationBlock q;
            vamp::FloatVector<rake, 7> rTeB;
            vamp::FloatVector<rake, 7> wTrB;
            vamp::FloatVector<rake, 6> lbB;
            vamp::FloatVector<rake, 6> ubB;
            const size_t size_per_eef = 7 + 7 + 6 + 6;

            auto operator[](size_t index) const
            {
                if (index < Robot::dimension)  // q
                {
                    return q[index];
                }

                size_t index_e = index - Robot::dimension;

                if (index_e < 7)  // rtE
                {
                    return rTeB[index_e];
                }

                else if (index_e >= 7 && index_e < (2 * 7))
                {
                    return wTrB[index_e - 7];
                }

                else if (index_e >= (2 * 7) && index_e < (2 * 7 + 6))
                {
                    return lbB[index_e - (2 * 7)];
                }

                else if (index_e >= (2 * 7 + 6) && index_e < (2 * 7 + 6 * 2))
                {
                    return ubB[index_e - (2 * 7 + 6)];
                }
                else
                    return q[0];
            }
        };

        TSRComputeInput tsr_function_inp;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, 6 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 6> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * Robot::dimension &&
                    index < 6 * Robot::dimension + 6)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 6 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 6 * Robot::dimension &&
                    index < 6 * Robot::dimension + 6)
                {
                    return err[index - 6 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, 6 + 6 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 6; i++)
                {
                    err[i] = y[6 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 6 * Robot::dimension; i++)
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
        SETaskSpaceConstraint(
            const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference,  // rTe
            const Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world,         // wTr
            const std::pair<std::array<float, 6>, std::array<float, 6>> bounds)
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
            for(auto i=0U; i < 6 * Robot::dimension; i++){
                if(i % Robot::dimension == 0)
                    std::cout << std::endl << i / Robot::dimension << " : ";
                std::cout << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            for(auto i=0U; i < 6; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q)
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            Robot::template single_eef_tsr_error<rake>(tsr_function_inp, jac_proj_inp);

            const size_t jac_offset = 6 * Robot::dimension;
            for (size_t i = 0; i < 6; i++)
            {
                jac_proj_inp[i + jac_offset] =
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.lbB[i]).min(0.F) +
                    (jac_proj_inp[i + jac_offset] - tsr_function_inp.ubB[i]).max(0.F);
            }
            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6; i++)
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
                    Robot::template solve_single_eef_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_single_eef_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_single_eef_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
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

                // if (q_dist_from_prev.test_all_less_equal(0.00001F))  // if i make no forward progress
                // {
                //     break;
                // }

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
            std::cout << "Num projection steps : " << project_iter << " ";
            // std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;

            return success;
        }


    };


}  // namespace vamp::planning