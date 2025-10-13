#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>
#include <vamp/vector/eigen.hh>
#include <vamp/vector/math.hh>
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
            ConfigurationBlock q_old;


            struct ErrGradOut {
                vamp::FloatVector<rake, 7> grad; // jacobian
                vamp::FloatVector<rake, 6> err; // error vector

                auto &operator[](size_t index) {
                    if (index < 7)
                        return grad[index];
                    else if (index >= 7 && index < 13)
                        return err[index - 7];
                    else
                        return grad[0];
                }

                const auto operator[](size_t index) const {
                    if (index < 7)
                        return grad[index];
                    else if (index >= 7 && index < 13)
                        return err[index - 7];
                    else
                        return grad[0];
                }


                ErrGradOut& operator=(vamp::FloatVector<rake, 6 + 7> y) {
                    for(auto i=0U; i < 7; i++)
                        grad[i] = y[i];
                    for(auto i=0U; i < 6; i++)
                        err[i] = y[i + 7];
                    return *this;
                }


            };
            ErrGradOut grad_err;


            struct QpErrOut {
                vamp::FloatVector<rake, 7> q_proj; // jacobian
                vamp::FloatVector<rake, 6> err; // error vector

                auto &operator[](size_t index) {
                    if (index < 7)
                        return q_proj[index];
                    else if (index >= 7 && index < 13)
                        return err[index - 7];
                    else
                        return q_proj[0];
                }

                const auto operator[](size_t index) const {
                    if (index < 7)
                        return q_proj[index];
                    else if (index >= 7 && index < 13)
                        return err[index - 7];
                    else
                        return q_proj[0];
                }


                QpErrOut& operator=(vamp::FloatVector<rake, 6 + 7> y) {
                    for(auto i=0U; i < 7; i++)
                        q_proj[i] = y[i];
                    for(auto i=0U; i < 6; i++)
                        err[i] = y[i + 7];
                    return *this;
                }


            };
            QpErrOut qproj_err;


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

        const auto print_robot_tsr_error(const ConfigurationBlock &q)
        {
            // std::cout << "-------- Printing ------- " << std::endl;
            auto dist = distanceToConstraint(q);
            // for(auto i=0U; i < 42; i++)
            //     std::cout << jac_proj_inp.J[{i, 0}] << " ";
            // std::cout << std::endl;
            // for(auto i=0U; i < 6; i++)
            //     std::cout << jac_proj_inp.err[{i, 0}] << " ";
            // std::cout << std::endl;
            projectStepJt(q, q_old, true);
            // std::cout << "-------- Printed ------- " << std::endl;

        }

        const auto distanceToConstraint(const ConfigurationBlock &q)
        {
            for (int i=0U; i < Robot::dimension; i++)
                tsr_function_inp.q[i] = q[i];
            
            Robot::tsr_function_ori_blend_simd(tsr_function_inp, jac_proj_inp);

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


        const auto gradient_and_error(const ConfigurationBlock &q)
        {
            // std::cout << "-------- Printing g&e------- " << std::endl;
            for (int i=0U; i < 7; i++)
                tsr_function_inp.q[i] = q[i];

            Robot::full_tsr_project(tsr_function_inp, qproj_err);

            // for(auto i=0U; i < 6; i++)
            //     std::cout << grad_err.err[{i, 0}] << " ";
            // std::cout << std::endl;
            // for(auto i=0U; i < 7; i++)
            //     std::cout << grad_err.grad[{i, 0}] << " ";
            // std::cout << std::endl;



            auto d = (grad_err[0 + 7] * grad_err[0 + 7] +
                      grad_err[1 + 7] * grad_err[1 + 7] +
                      grad_err[2 + 7] * grad_err[2 + 7] + 
                      grad_err[3 + 7] * grad_err[3 + 7] + 
                      grad_err[4 + 7] * grad_err[4 + 7] + 
                      grad_err[5 + 7] * grad_err[5 + 7]
                    ).abs();
            return d;
            // std::cout << "-------- Printed g&e------- " << std::endl;
        }



        auto jacobian_solve_config_jt(const JacobianProjectInp &x) noexcept
        {
            /**
            @brief Calculates grad ~= J.t * error

            @param x - A container for J and E with the [] operator
            @return grad
            */
            vamp::FloatVector<rake, 7> y;
            // y[0] = x[5] * x[47] + x[4] * x[46] + x[3] * x[45] + x[2] * x[44] + x[1] * x[43] + x[0] * x[42];
            // y[1] = x[11] * x[47] + x[10] * x[46] + x[9] * x[45] + x[8] * x[44] + x[7] * x[43] + x[6] * x[42];
            // y[2] =
            //     x[17] * x[47] + x[16] * x[46] + x[15] * x[45] + x[14] * x[44] + x[13] * x[43] + x[12] * x[42];
            // y[3] =
            //     x[23] * x[47] + x[22] * x[46] + x[21] * x[45] + x[20] * x[44] + x[19] * x[43] + x[18] * x[42];
            // y[4] =
            //     x[29] * x[47] + x[28] * x[46] + x[27] * x[45] + x[26] * x[44] + x[25] * x[43] + x[24] * x[42];
            // y[5] =
            //     x[35] * x[47] + x[34] * x[46] + x[33] * x[45] + x[32] * x[44] + x[31] * x[43] + x[30] * x[42];
            // y[6] =
            //     x[41] * x[47] + x[40] * x[46] + x[39] * x[45] + x[38] * x[44] + x[37] * x[43] + x[36] * x[42];

            y[0] =
                x[35] * x[47] + x[28] * x[46] + x[21] * x[45] + x[14] * x[44] + x[7] * x[43] + x[0] * x[42];
            y[1] =
                x[36] * x[47] + x[29] * x[46] + x[22] * x[45] + x[15] * x[44] + x[8] * x[43] + x[1] * x[42];
            y[2] =
                x[37] * x[47] + x[30] * x[46] + x[23] * x[45] + x[16] * x[44] + x[9] * x[43] + x[2] * x[42];
            y[3] =
                x[38] * x[47] + x[31] * x[46] + x[24] * x[45] + x[17] * x[44] + x[10] * x[43] + x[3] * x[42];
            y[4] =
                x[39] * x[47] + x[32] * x[46] + x[25] * x[45] + x[18] * x[44] + x[11] * x[43] + x[4] * x[42];
            y[5] =
                x[40] * x[47] + x[33] * x[46] + x[26] * x[45] + x[19] * x[44] + x[12] * x[43] + x[5] * x[42];
            y[6] =
                x[41] * x[47] + x[34] * x[46] + x[27] * x[45] + x[20] * x[44] + x[13] * x[43] + x[6] * x[42];            
            return y;
        }

        auto jacobian_solve_config_cholesky(const JacobianProjectInp &x) noexcept
        {
            /**
            @brief Calculates grad = J.t (JJ.t + aI)-1 * error using a Cholesky decomp

            @param x - A container for J and E with the [] operator
            @return grad
            */

            FloatVector<8, 34> v;
            FloatVector<8, 7> y;
            v[0] = (
                0.0001 + x[0] * x[0] + x[7] * x[7] + x[14] * x[14] + x[21] * x[21] + x[28] * x[28] +
                x[35] * x[35]).sqrt();
            v[1] = (x[35] * x[47] + x[28] * x[46] + x[21] * x[45] + x[14] * x[44] + x[7] * x[43] +
                    x[0] * x[42]) /
                   v[0];
            v[2] =
                (x[1] * x[0] + x[8] * x[7] + x[15] * x[14] + x[22] * x[21] + x[29] * x[28] + x[36] * x[35]) /
                v[0];
            v[3] = (
                0.0001 + x[1] * x[1] + x[8] * x[8] + x[15] * x[15] + x[22] * x[22] + x[29] * x[29] +
                x[36] * x[36] - v[2] * v[2]).sqrt();
            v[4] = (x[36] * x[47] + x[29] * x[46] + x[22] * x[45] + x[15] * x[44] + x[8] * x[43] +
                    x[1] * x[42] - v[2] * v[1]) /
                   v[3];
            v[5] =
                (x[2] * x[0] + x[9] * x[7] + x[16] * x[14] + x[23] * x[21] + x[30] * x[28] + x[37] * x[35]) /
                v[0];
            v[6] = (x[2] * x[1] + x[9] * x[8] + x[16] * x[15] + x[23] * x[22] + x[30] * x[29] +
                    x[37] * x[36] - v[5] * v[2]) /
                   v[3];
            v[7] = (
                0.0001 + x[2] * x[2] + x[9] * x[9] + x[16] * x[16] + x[23] * x[23] + x[30] * x[30] +
                x[37] * x[37] - v[5] * v[5] - v[6] * v[6]).sqrt();
            v[8] = (x[37] * x[47] + x[30] * x[46] + x[23] * x[45] + x[16] * x[44] + x[9] * x[43] +
                    x[2] * x[42] - v[5] * v[1] - v[6] * v[4]) /
                   v[7];
            v[9] =
                (x[3] * x[0] + x[10] * x[7] + x[17] * x[14] + x[24] * x[21] + x[31] * x[28] + x[38] * x[35]) /
                v[0];
            v[10] = (x[3] * x[1] + x[10] * x[8] + x[17] * x[15] + x[24] * x[22] + x[31] * x[29] +
                     x[38] * x[36] - v[9] * v[2]) /
                    v[3];
            v[11] = (x[3] * x[2] + x[10] * x[9] + x[17] * x[16] + x[24] * x[23] + x[31] * x[30] +
                     x[38] * x[37] - v[9] * v[5] - v[10] * v[6]) /
                    v[7];
            v[12] = (
                0.0001 + x[3] * x[3] + x[10] * x[10] + x[17] * x[17] + x[24] * x[24] + x[31] * x[31] +
                x[38] * x[38] - v[9] * v[9] - v[10] * v[10] - v[11] * v[11]).sqrt();
            v[13] = (x[38] * x[47] + x[31] * x[46] + x[24] * x[45] + x[17] * x[44] + x[10] * x[43] +
                     x[3] * x[42] - v[9] * v[1] - v[10] * v[4] - v[11] * v[8]) /
                    v[12];
            v[14] =
                (x[4] * x[0] + x[11] * x[7] + x[18] * x[14] + x[25] * x[21] + x[32] * x[28] + x[39] * x[35]) /
                v[0];
            v[15] = (x[4] * x[1] + x[11] * x[8] + x[18] * x[15] + x[25] * x[22] + x[32] * x[29] +
                     x[39] * x[36] - v[14] * v[2]) /
                    v[3];
            v[16] = (x[4] * x[2] + x[11] * x[9] + x[18] * x[16] + x[25] * x[23] + x[32] * x[30] +
                     x[39] * x[37] - v[14] * v[5] - v[15] * v[6]) /
                    v[7];
            v[17] = (x[4] * x[3] + x[11] * x[10] + x[18] * x[17] + x[25] * x[24] + x[32] * x[31] +
                     x[39] * x[38] - v[14] * v[9] - v[15] * v[10] - v[16] * v[11]) /
                    v[12];
            v[18] = (
                0.0001 + x[4] * x[4] + x[11] * x[11] + x[18] * x[18] + x[25] * x[25] + x[32] * x[32] +
                x[39] * x[39] - v[14] * v[14] - v[15] * v[15] - v[16] * v[16] - v[17] * v[17]).sqrt();
            v[19] = (x[39] * x[47] + x[32] * x[46] + x[25] * x[45] + x[18] * x[44] + x[11] * x[43] +
                     x[4] * x[42] - v[14] * v[1] - v[15] * v[4] - v[16] * v[8] - v[17] * v[13]) /
                    v[18];
            v[20] =
                (x[5] * x[0] + x[12] * x[7] + x[19] * x[14] + x[26] * x[21] + x[33] * x[28] + x[40] * x[35]) /
                v[0];
            v[21] = (x[5] * x[1] + x[12] * x[8] + x[19] * x[15] + x[26] * x[22] + x[33] * x[29] +
                     x[40] * x[36] - v[20] * v[2]) /
                    v[3];
            v[22] = (x[5] * x[2] + x[12] * x[9] + x[19] * x[16] + x[26] * x[23] + x[33] * x[30] +
                     x[40] * x[37] - v[20] * v[5] - v[21] * v[6]) /
                    v[7];
            v[23] = (x[5] * x[3] + x[12] * x[10] + x[19] * x[17] + x[26] * x[24] + x[33] * x[31] +
                     x[40] * x[38] - v[20] * v[9] - v[21] * v[10] - v[22] * v[11]) /
                    v[12];
            v[24] = (x[5] * x[4] + x[12] * x[11] + x[19] * x[18] + x[26] * x[25] + x[33] * x[32] +
                     x[40] * x[39] - v[20] * v[14] - v[21] * v[15] - v[22] * v[16] - v[23] * v[17]) /
                    v[18];
            v[25] = (
                0.0001 + x[5] * x[5] + x[12] * x[12] + x[19] * x[19] + x[26] * x[26] + x[33] * x[33] +
                x[40] * x[40] - v[20] * v[20] - v[21] * v[21] - v[22] * v[22] - v[23] * v[23] -
                v[24] * v[24]).sqrt();
            v[26] =
                (x[40] * x[47] + x[33] * x[46] + x[26] * x[45] + x[19] * x[44] + x[12] * x[43] +
                 x[5] * x[42] - v[20] * v[1] - v[21] * v[4] - v[22] * v[8] - v[23] * v[13] - v[24] * v[19]) /
                v[25];
            v[27] =
                (x[6] * x[0] + x[13] * x[7] + x[20] * x[14] + x[27] * x[21] + x[34] * x[28] + x[41] * x[35]) /
                v[0];
            v[28] = (x[6] * x[1] + x[13] * x[8] + x[20] * x[15] + x[27] * x[22] + x[34] * x[29] +
                     x[41] * x[36] - v[27] * v[2]) /
                    v[3];
            v[29] = (x[6] * x[2] + x[13] * x[9] + x[20] * x[16] + x[27] * x[23] + x[34] * x[30] +
                     x[41] * x[37] - v[27] * v[5] - v[28] * v[6]) /
                    v[7];
            v[30] = (x[6] * x[3] + x[13] * x[10] + x[20] * x[17] + x[27] * x[24] + x[34] * x[31] +
                     x[41] * x[38] - v[27] * v[9] - v[28] * v[10] - v[29] * v[11]) /
                    v[12];
            v[31] = (x[6] * x[4] + x[13] * x[11] + x[20] * x[18] + x[27] * x[25] + x[34] * x[32] +
                     x[41] * x[39] - v[27] * v[14] - v[28] * v[15] - v[29] * v[16] - v[30] * v[17]) /
                    v[18];
            v[32] =
                (x[6] * x[5] + x[13] * x[12] + x[20] * x[19] + x[27] * x[26] + x[34] * x[33] + x[41] * x[40] -
                 v[27] * v[20] - v[28] * v[21] - v[29] * v[22] - v[30] * v[23] - v[31] * v[24]) /
                v[25];
            v[33] = (
                0.0001 + x[6] * x[6] + x[13] * x[13] + x[20] * x[20] + x[27] * x[27] + x[34] * x[34] +
                x[41] * x[41] - v[27] * v[27] - v[28] * v[28] - v[29] * v[29] - v[30] * v[30] -
                v[31] * v[31] - v[32] * v[32]).sqrt();
            y[6] = ((x[41] * x[47] + x[34] * x[46] + x[27] * x[45] + x[20] * x[44] + x[13] * x[43] +
                     x[6] * x[42] - v[27] * v[1] - v[28] * v[4] - v[29] * v[8] - v[30] * v[13] -
                     v[31] * v[19] - v[32] * v[26]) /
                    v[33]) /
                   v[33];
            y[5] = (v[26] - v[32] * y[6]) / v[25];
            y[4] = (v[19] - v[24] * y[5] - v[31] * y[6]) / v[18];
            y[3] = (v[13] - v[17] * y[4] - v[23] * y[5] - v[30] * y[6]) / v[12];
            y[2] = (v[8] - v[11] * y[3] - v[16] * y[4] - v[22] * y[5] - v[29] * y[6]) / v[7];
            y[1] = (v[4] - v[6] * y[2] - v[10] * y[3] - v[15] * y[4] - v[21] * y[5] - v[28] * y[6]) / v[3];
            y[0] = (v[1] - v[2] * y[1] - v[5] * y[2] - v[9] * y[3] - v[14] * y[4] - v[20] * y[5] -
                    v[27] * y[6]) /
                   v[0];

            return y;
        }



        void integrateJointConfiguration(const ConfigurationBlock &q, ConfigurationBlock &q_new, const ConfigurationBlock &grad)
        {
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i] - grad[i];
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

        auto projectStepJt(const ConfigurationBlock &q, ConfigurationBlock &q_new, bool update_q = true)
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "dist base " << dist << std::endl;
            if (update_q)
            {
                auto grad = jacobian_solve_config_jt(jac_proj_inp);
                // for(auto i=0U; i < 7; i++)
                //     std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;


                integrateJointConfigurationJt(q, q_new, grad);
            }
            return dist;
        }

        auto projectStepDirect(const ConfigurationBlock &q, ConfigurationBlock &q_new, bool update_q = true)
        {
            auto dist = gradient_and_error(q);
            if (update_q){
                for(auto i=0U; i < Robot::dimension; i++)
                    q_new[i] = qproj_err[i];
            }
            //     integrateJointConfiguration(q, q_new, grad_err.grad);
            return dist;
        }

        void integrateJointConfigurationJt(const ConfigurationBlock &q, ConfigurationBlock &q_new, const ConfigurationBlock &grad)
        {
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i] - grad[i];
            Robot::descale_configuration_block(q_new);
            q_new = q_new.clamp(0.F, 1.F);
            Robot::scale_configuration_block(q_new);
        }

        bool project(const ConfigurationBlock &q, ConfigurationBlock &q_new, float max_q_dist = 5.0)
        {
            bool success = false;
            auto dist = distanceToConstraint(q);

            size_t project_iter = 0;
            for (auto i = 0U; i < Robot::dimension; i++)
            {
                q_new[i] = q[i];
                q_old[i] = q[i];
            }

            while ((project_iter < 100) and (not dist.test_all_less_equal(0.0001F)))
            {
                dist = projectStepJt(q_old, q_new, true);
                auto q_dist = (q_new[0] - q_old[0]) * (q_new[0] - q_old[0]);
                for (auto i = 1U; i < Robot::dimension; i++)
                    q_dist = q_dist + (q_new[i] - q_old[i]) * (q_new[i] - q_old[i]);

                if (q_dist.test_all_less_equal(0.00001F)) // if i make no forward progress
                    break;

                if (q_dist.test_any_greater_equal(4 * max_q_dist * max_q_dist)) // from triangle inequality
                    break;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //     q_old[i] = q_new[i];
                q_old = q_new + 0.0;
                project_iter += 1;
            }

            if (dist.test_all_less_equal(0.0001F))
                success = true;

            // std::cout << "Num steps : " << project_iter << " and success : " << success << std::endl;

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
// TODO (siyer):
// 1. Make it robot agnostic by templatizing on robot type
// 2. Try out 50.0 and 20.0 for pos and ori constraints to check for faster convergence
// 3. Benchmark Jt, Direct and projectStep on a bunch of examples
// 4. Tune multiple knobs -- steps of projection, max step size, dist threshold, method, cbirrt connect, triangle ineqs -- hyperplan smac3
// 5. Clean up cbirrt connect
// 6. cbirrt connect step fix
// 7. try out ldlt() instead of cholesky

// TODO (siyer)
// 1. Tasks - maze, door etc
// 2. Bimanual -- think about in-hand-pose