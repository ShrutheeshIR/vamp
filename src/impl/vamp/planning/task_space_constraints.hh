#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
// #include <vamp/vector/interface.hh>




namespace vamp::planning
{
    template <typename Robot>
    class RobotConstraint
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;

        protected:
            const double tolerance = 1e-2;
            const size_t max_project_iters = 10;

        // public:
        //     void integrateJointConfiguration(const ConfigurationArray &q, ConfigurationArray &q_new, const Eigen::Matrix<float, Eigen::Dynamic, 1> &update_vector)
        //     {
        //         for (size_t i = 0; i < q.size(); i ++)
        //             // need to adjust joint limits
        //             q_new[i] = q[i] + update_vector(i);
        //     }

        //     double projectStepOld(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
        //     {
        //         Eigen::Matrix<float, 6, Eigen::Dynamic> J;
        //         Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

        //         Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
        //         const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

        //         if (update_q)
        //         {
        //             // need to convert jacobian into ref frame
        //             const Eigen::Matrix<float, Eigen::Dynamic, 1> update = -J.transpose() * (J * J.transpose() + 1e-6 * Eigen::Matrix<float, 6, 6>::Identity()).ldlt().solve(distance);
        //             integrateJointConfiguration(q, q_new, update);
        //         }
        //         return distance.squaredNorm();

        //     }


        //     double projectStepJT(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
        //     {
        //         Eigen::Matrix<float, 6, Eigen::Dynamic> J;
        //         Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

        //         Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
        //         const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

        //         if (update_q)
        //         {
        //             // need to convert jacobian into ref frame
        //             const Eigen::Matrix<float, Eigen::Dynamic, 1> update = -J.transpose() * distance;
        //             integrateJointConfiguration(q, q_new, update);
        //         }
        //         return distance.squaredNorm();

        //     }

        //     // project with vamp inverse
        //     double projectStep(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
        //     {
        //         Eigen::Matrix<float, 6, Eigen::Dynamic> J;
        //         Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;
        //         Eigen::Matrix<float, 7, 1> grad;

        //         Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
        //         const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

        //         std::array<float, 6> dist_arr;
        //         std::copy(distance.data(), distance.data() + distance.size(), dist_arr.begin());



        //         if (update_q)
        //         {
        //             // ;
                    
        //             Robot::jacobian_solve(q, dist_arr, grad);
        //             // std::cout << grad << std::endl;
        //             integrateJointConfiguration(q, q_new, -grad);
        //         }
        //         return distance.squaredNorm();

        //     }


        //     bool project(const ConfigurationArray &q, ConfigurationArray &q_new)
        //     {
        //         bool success = false;
        //         // double distance = projectStep(q, q_new, false);
        //         // size_t project_iter = 0;
        //         // // std::cout << distance << " " << std::endl;
        //         // while ((project_iter < max_project_iters) && (distance > tolerance))
        //         // {
        //         //     distance = projectStepJT(q_new, q_new, true);
        //         //     // std::cout << distance << " " << std::endl;
        //         // }
        //         // if (distance < tolerance)
        //         // {
        //         //     // std::cout << distance << " " << std::endl;
        //         //     const auto fk = Robot::eefk(q_new);
        //         //     std::cout << fk.matrix()(2,3) << std::endl;

        //         //     success = true;
        //         // }
        //         return success;
                    
        //     }

        //     bool isValid(const ConfigurationArray &q)
        //     {
        //         const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose = Robot::eefk(q);
        //         const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

        //         return distance.squaredNorm() < tolerance;
        //     }

        //     virtual const Eigen::VectorXf distanceToConstraintEEF(Eigen::Transform<float, 3, Eigen::Isometry> computed_eef_pose_world_frame) = 0;
            
    };

    // We will work with the assumption that all the transforms are given to us
    template <typename Robot, std::size_t rake>
    class TaskSpaceConstraint : public RobotConstraint<Robot>
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        protected:

            const size_t se3_size = 16;

            // 6 is for SE3 space 
            Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference;
            Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world;

            // std::pair<vamp::FloatVector<6>, vamp::FloatVector<6>> bounds;
            std::pair<std::array<float, 6>, std::array<float, 6>> bounds; 

            struct TSRFuncInput {
                ConfigurationBlock q;
                vamp::FloatVector<rake, 7>rTeB;
                vamp::FloatVector<rake, 7> wTrB;
                vamp::FloatVector<rake, 6> lbB;
                vamp::FloatVector<rake, 6> ubB;
                
                auto operator[](size_t index) const {
                    if (index < 7) // q
                        return q[index];

                    if (index >= 7 && index < (2 * 7)) // rtE
                        return rTeB[index - 7];

                    else if (index >= (2 * 7) && index < (3 * 7))
                        return wTrB[index - (2 * 7)];


                    else if (index >= (3 * 7) && index < (3 * 7 + 6))
                        return lbB[index - (3 * 7)];

                    else if (index >= (3 * 7 + 6) && index < (3 * 7 + 6 * 2))
                        return ubB[index - (3 * 7 + 6)];
                    else
                        return rTeB[0]; // to prevent complaining
                }
            };
            TSRFuncInput tsr_function_inp;


            struct JacobianProjectInp {
                vamp::FloatVector<rake, 6 * 7>J; // jacobian
                vamp::FloatVector<rake, 6> err; // error vector

                auto &operator[](size_t index) {
                    if (index < 42)
                        return J[index];
                    else if (index >= 42 && index < 48)
                        return err[index - 42];
                    else
                        return err[0];
                }

                const auto operator[](size_t index) const {
                    if (index < 42)
                        return J[index];
                    else if (index >= 42 && index < 48)
                        return err[index - 42];
                    else
                        return err[0];
                }


                JacobianProjectInp& operator=(vamp::FloatVector<rake, 6 + 6 * 7> y) {
                    for(auto i=0U; i < 6; i++)
                        err[i] = y[42 + i];
                    for(auto i=0U; i < 42; i++)
                        J[i] = y[42];
                    return *this;
                }


            };
            JacobianProjectInp jac_proj_inp;

        template <std::size_t dim>
        inline static auto assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
        {
            for (auto i = 0U; i < dim; i++)
                dest[i] = src[i];
        }


        public:

            TaskSpaceConstraint(
                const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference, // rTe
                const Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world, // wTr
                const std::pair<std::array<float, 6>, std::array<float, 6>> bounds
            ) : eef_pose_w_ref_reference(eef_pose_w_ref_reference),  ref_frame_w_world(ref_frame_w_world), bounds(bounds)
            {


                Eigen::Quaternion<float> q1(eef_pose_w_ref_reference.linear());
                std::array<float, 7> transform1 = {q1.w(), q1.x(), q1.y(), q1.z(), eef_pose_w_ref_reference.translation().x(), eef_pose_w_ref_reference.translation().y(), eef_pose_w_ref_reference.translation().z()};

                Eigen::Quaternion<float> q2(ref_frame_w_world.linear());
                std::array<float, 7> transform2 = {q2.w(), q2.x(), q2.y(), q2.z(), ref_frame_w_world.translation().x(), ref_frame_w_world.translation().y(), ref_frame_w_world.translation().z()};
                assignBlock<7>(transform1, tsr_function_inp.rTeB);
                assignBlock<7>(transform2, tsr_function_inp.wTrB);
                
                assignBlock<6>(bounds.first, tsr_function_inp.lbB);
                assignBlock<6>(bounds.second, tsr_function_inp.ubB);
            }

            const auto function(const ConfigurationBlock &q) {
                return distanceToConstraint(q);
            }

        const auto distanceToConstraint(const ConfigurationBlock &q)
        {
            for (int i=0U; i < 7; i++)
                tsr_function_inp.q[i] = q[i];

            Robot::tsr_function_jac(tsr_function_inp, jac_proj_inp);

            for(int i=0U; i < 6;i++)
            {
                jac_proj_inp[i + 42] = (jac_proj_inp[i + 42] - tsr_function_inp[ 21 + i]).min(0.F) 
                                    + (jac_proj_inp[i + 42] - tsr_function_inp[ 27 + i]).max(0.F);
            }

            auto d =(jac_proj_inp[0 + 42] * jac_proj_inp[0 + 42] +
                      jac_proj_inp[1 + 42] * jac_proj_inp[1 + 42] +
                      jac_proj_inp[2 + 42] * jac_proj_inp[2 + 42] + 
                      jac_proj_inp[3 + 42] * jac_proj_inp[3 + 42] + 
                      jac_proj_inp[4 + 42] * jac_proj_inp[4 + 42] + 
                      jac_proj_inp[5 + 42] * jac_proj_inp[5 + 42]
                    ).abs();
            return d;
        }


        auto jacobian_solve_config_jt(const JacobianProjectInp &x) noexcept
        {
            /**
            @brief Calculates grad ~= J.t * error

            @param x - A container for J and E with the [] operator
            @return grad
            */
            vamp::FloatVector<rake, 7> y;
            y[0] = x[5] * x[47] + x[4] * x[46] + x[3] * x[45] + x[2] * x[44] + x[1] * x[43] + x[0] * x[42];
            y[1] = x[11] * x[47] + x[10] * x[46] + x[9] * x[45] + x[8] * x[44] + x[7] * x[43] + x[6] * x[42];
            y[2] =
                x[17] * x[47] + x[16] * x[46] + x[15] * x[45] + x[14] * x[44] + x[13] * x[43] + x[12] * x[42];
            y[3] =
                x[23] * x[47] + x[22] * x[46] + x[21] * x[45] + x[20] * x[44] + x[19] * x[43] + x[18] * x[42];
            y[4] =
                x[29] * x[47] + x[28] * x[46] + x[27] * x[45] + x[26] * x[44] + x[25] * x[43] + x[24] * x[42];
            y[5] =
                x[35] * x[47] + x[34] * x[46] + x[33] * x[45] + x[32] * x[44] + x[31] * x[43] + x[30] * x[42];
            y[6] =
                x[41] * x[47] + x[40] * x[46] + x[39] * x[45] + x[38] * x[44] + x[37] * x[43] + x[36] * x[42];
            
            return y;
        }

        auto jacobian_solve_config_cholesky(const JacobianProjectInp &x) noexcept
        {
            /**
            @brief Calculates grad = J.t (JJ.t + aI)-1 * error using a Cholesky decomp

            @param x - A container for J and E with the [] operator
            @return grad
            */

            FloatVector<rake, 21> v;
            FloatVector<rake, 7> y;
            v[0] = (
                0.0001 + x[0] * x[0] + x[6] * x[6] + x[12] * x[12] + x[18] * x[18] + x[24] * x[24] +
                x[30] * x[30] + x[36] * x[36]).sqrt();
            v[1] = (x[5] * x[0] + x[11] * x[6] + x[17] * x[12] + x[23] * x[18] + x[29] * x[24] +
                    x[35] * x[30] + x[41] * x[36]) /
                   v[0];
            v[2] = (x[1] * x[0] + x[7] * x[6] + x[13] * x[12] + x[19] * x[18] + x[25] * x[24] +
                    x[31] * x[30] + x[37] * x[36]) /
                   v[0];
            v[3] = (
                0.0001 + x[1] * x[1] + x[7] * x[7] + x[13] * x[13] + x[19] * x[19] + x[25] * x[25] +
                x[31] * x[31] + x[37] * x[37] - v[2] * v[2]).sqrt();
            v[4] = (x[5] * x[1] + x[11] * x[7] + x[17] * x[13] + x[23] * x[19] + x[29] * x[25] +
                    x[35] * x[31] + x[41] * x[37] - v[1] * v[2]) /
                   v[3];
            v[5] = (x[2] * x[0] + x[8] * x[6] + x[14] * x[12] + x[20] * x[18] + x[26] * x[24] +
                    x[32] * x[30] + x[38] * x[36]) /
                   v[0];
            v[6] = (x[2] * x[1] + x[8] * x[7] + x[14] * x[13] + x[20] * x[19] + x[26] * x[25] +
                    x[32] * x[31] + x[38] * x[37] - v[5] * v[2]) /
                   v[3];
            v[7] = (
                0.0001 + x[2] * x[2] + x[8] * x[8] + x[14] * x[14] + x[20] * x[20] + x[26] * x[26] +
                x[32] * x[32] + x[38] * x[38] - v[5] * v[5] - v[6] * v[6]).sqrt();
            v[8] = (x[5] * x[2] + x[11] * x[8] + x[17] * x[14] + x[23] * x[20] + x[29] * x[26] +
                    x[35] * x[32] + x[41] * x[38] - v[1] * v[5] - v[4] * v[6]) /
                   v[7];
            v[9] = (x[3] * x[0] + x[9] * x[6] + x[15] * x[12] + x[21] * x[18] + x[27] * x[24] +
                    x[33] * x[30] + x[39] * x[36]) /
                   v[0];
            v[10] = (x[3] * x[1] + x[9] * x[7] + x[15] * x[13] + x[21] * x[19] + x[27] * x[25] +
                     x[33] * x[31] + x[39] * x[37] - v[9] * v[2]) /
                    v[3];
            v[11] = (x[3] * x[2] + x[9] * x[8] + x[15] * x[14] + x[21] * x[20] + x[27] * x[26] +
                     x[33] * x[32] + x[39] * x[38] - v[9] * v[5] - v[10] * v[6]) /
                    v[7];
            v[12] = (
                0.0001 + x[3] * x[3] + x[9] * x[9] + x[15] * x[15] + x[21] * x[21] + x[27] * x[27] +
                x[33] * x[33] + x[39] * x[39] - v[9] * v[9] - v[10] * v[10] - v[11] * v[11]).sqrt();
            v[13] = (x[5] * x[3] + x[11] * x[9] + x[17] * x[15] + x[23] * x[21] + x[29] * x[27] +
                     x[35] * x[33] + x[41] * x[39] - v[1] * v[9] - v[4] * v[10] - v[8] * v[11]) /
                    v[12];
            v[14] = (x[4] * x[0] + x[10] * x[6] + x[16] * x[12] + x[22] * x[18] + x[28] * x[24] +
                     x[34] * x[30] + x[40] * x[36]) /
                    v[0];
            v[15] = (x[4] * x[1] + x[10] * x[7] + x[16] * x[13] + x[22] * x[19] + x[28] * x[25] +
                     x[34] * x[31] + x[40] * x[37] - v[14] * v[2]) /
                    v[3];
            v[16] = (x[4] * x[2] + x[10] * x[8] + x[16] * x[14] + x[22] * x[20] + x[28] * x[26] +
                     x[34] * x[32] + x[40] * x[38] - v[14] * v[5] - v[15] * v[6]) /
                    v[7];
            v[17] = (x[4] * x[3] + x[10] * x[9] + x[16] * x[15] + x[22] * x[21] + x[28] * x[27] +
                     x[34] * x[33] + x[40] * x[39] - v[14] * v[9] - v[15] * v[10] - v[16] * v[11]) /
                    v[12];
            v[18] = (
                0.0001 + x[4] * x[4] + x[10] * x[10] + x[16] * x[16] + x[22] * x[22] + x[28] * x[28] +
                x[34] * x[34] + x[40] * x[40] - v[14] * v[14] - v[15] * v[15] - v[16] * v[16] -
                v[17] * v[17]).sqrt();
            v[19] =
                (x[5] * x[4] + x[11] * x[10] + x[17] * x[16] + x[23] * x[22] + x[29] * x[28] + x[35] * x[34] +
                 x[41] * x[40] - v[1] * v[14] - v[4] * v[15] - v[8] * v[16] - v[13] * v[17]) /
                v[18];
            v[20] = (
                0.0001 + x[5] * x[5] + x[11] * x[11] + x[17] * x[17] + x[23] * x[23] + x[29] * x[29] +
                x[35] * x[35] + x[41] * x[41] - v[1] * v[1] - v[4] * v[4] - v[8] * v[8] - v[13] * v[13] -
                v[19] * v[19]).sqrt();
            v[20] = (x[47] - v[1] * x[42] - v[4] * x[43] - v[8] * x[44] - v[13] * x[45] - v[19] * x[46] -
                     v[20] * x[47]) /
                    v[20];
            v[18] = (x[46] - v[14] * x[42] - v[15] * x[43] - v[16] * x[44] - v[17] * x[45] - v[18] * x[46]) /
                    v[18];
            v[12] = (x[45] - v[9] * x[42] - v[10] * x[43] - v[11] * x[44] - v[12] * x[45]) / v[12];
            v[7] = (x[44] - v[5] * x[42] - v[6] * x[43] - v[7] * x[44]) / v[7];
            v[3] = (x[43] - v[2] * x[42] - v[3] * x[43]) / v[3];
            v[0] = (x[42] - v[0] * x[42]) / v[0];
            y[0] = x[5] * v[20] + x[4] * v[18] + x[3] * v[12] + x[2] * v[7] + x[1] * v[3] + x[0] * v[0];
            y[1] = x[11] * v[20] + x[10] * v[18] + x[9] * v[12] + x[8] * v[7] + x[7] * v[3] + x[6] * v[0];
            y[2] = x[17] * v[20] + x[16] * v[18] + x[15] * v[12] + x[14] * v[7] + x[13] * v[3] + x[12] * v[0];
            y[3] = x[23] * v[20] + x[22] * v[18] + x[21] * v[12] + x[20] * v[7] + x[19] * v[3] + x[18] * v[0];
            y[4] = x[29] * v[20] + x[28] * v[18] + x[27] * v[12] + x[26] * v[7] + x[25] * v[3] + x[24] * v[0];
            y[5] = x[35] * v[20] + x[34] * v[18] + x[33] * v[12] + x[32] * v[7] + x[31] * v[3] + x[30] * v[0];
            y[6] = x[41] * v[20] + x[40] * v[18] + x[39] * v[12] + x[38] * v[7] + x[37] * v[3] + x[36] * v[0];

            return y;
        }



        void integrateJointConfiguration(const ConfigurationBlock &q, ConfigurationBlock &q_new, const ConfigurationBlock &grad)
        {
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i] + grad[i];
            Robot::descale_configuration_block(q_new);
            q_new = q_new.clamp(0.F, 1.F);
            Robot::scale_configuration_block(q_new);
        }


        auto projectStep(const ConfigurationBlock &q, ConfigurationBlock &q_new, bool update_q = true)
        {
            auto dist = distanceToConstraint(q);
            if (update_q)
            {
                auto grad = jacobian_solve_config_cholesky(jac_proj_inp);
                integrateJointConfiguration(q, q_new, grad);
            }
            return dist;
        }



        bool project(const ConfigurationBlock &q, ConfigurationBlock &q_new)
        {
            bool success = false;
            auto dist = distanceToConstraint(q);

            size_t project_iter = 0;
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i];

            while ((project_iter < 1e3) and (not dist.test_all_less_equal(0.0001F)))
            {
                dist = projectStep(q_new, q_new, true);
                project_iter += 1;
            }

            if (dist.test_all_less_equal(0.0001F))
                success = true;

            // std::cout << "Num steps : " << project_iter << std::endl;

            // Robot::jacobian_eefk(q_new, tsr_distance_inp.wTeqB, jac_proj_inp.J);
            // std::cout << "FK after proj : " << tsr_distance_inp.wTeqB[12] <<", " << tsr_distance_inp.wTeqB[13] <<", " << tsr_distance_inp.wTeqB[14] << std::endl;
            // std::cout << "Full error is : " << jac_proj_inp.err << std::endl;
            // std::cout << "Projected config is : " << q_new << std::endl;
            // std::cout << "Initial config is : " << q << std::endl;

            // std::cout << "dist is : " << dist << std::endl;
            // std::cout << "success is : " << success << std::endl;

            return success;
                
        }

    };

}