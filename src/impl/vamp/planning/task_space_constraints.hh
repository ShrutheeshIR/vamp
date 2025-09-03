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
            Eigen::Transform<float, 3, Eigen::Isometry> world_frame_w_ref_frame;
            Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_world_reference;
            // std::pair<vamp::FloatVector<6>, vamp::FloatVector<6>> bounds;
            std::pair<std::array<float, 6>, std::array<float, 6>> bounds; 

            // creating std versions for block operations
            std::array<float, 16> rTe;
            std::array<float, 16> wTr;


            bool mComputeErrorFromCenter;



            // The following are internal data needed for computation
            struct TSRErrInp {
                vamp::FloatVector<rake, 16>rTeB;
                vamp::FloatVector<rake, 16> wTrB;
                vamp::FloatVector<rake, 16> wTeqB;
                vamp::FloatVector<rake, 6> lbB;
                vamp::FloatVector<rake, 6> ubB;
                
                auto operator[](size_t index) const {
                    if (index >= 0 && index < 16)
                        return rTeB[index];
                    else if (index >= 16 && index < 32)
                        return wTrB[index - 16];
                    else if (index >= 32 && index < 48)
                        return wTeqB[index - 32];
                    else if (index >= 48 && index < 54)
                        return lbB[index - 48];
                    else if (index >= 54 && index < 60)
                        return ubB[index - 54];
                    else
                        return rTeB[0]; // to prevent complaining
                }
            };
            TSRErrInp tsr_distance_inp;

            struct TSRFuncInput {
                ConfigurationBlock q;
                vamp::FloatVector<rake, 7>rTeB;
                vamp::FloatVector<rake, 7> wTrB;
                vamp::FloatVector<rake, 6> lbB;
                vamp::FloatVector<rake, 6> ubB;
                
                auto operator[](size_t index) const {
                    if (index >= 0 && index < 7) // q
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
                    if (index >= 0 && index < 42)
                        return J[index];
                    else if (index >= 42 && index < 48)
                        return err[index - 42];
                    else
                        return err[0];
                }

                const auto operator[](size_t index) const {
                    if (index >= 0 && index < 42)
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
                world_frame_w_ref_frame = ref_frame_w_world.inverse();
                eef_pose_w_world_reference = ref_frame_w_world * eef_pose_w_ref_reference;

                // rTe = eigen_to_vector(eef_pose_w_ref_reference.matrix().reshaped());
                // wTr = eigen_to_vector(ref_frame_w_world.matrix().reshaped());
                for (auto i = 0U; i < 4; i++)
                    for (auto j = 0U; j < 4; j++)
                        rTe[i*4 + j] = eef_pose_w_ref_reference.matrix()(j, i);


                for (auto i = 0U; i < 4; i++)
                    for (auto j = 0U; j < 4; j++)
                        wTr[i*4 + j] = ref_frame_w_world.matrix()(j, i);

                assignBlock<16>(rTe, tsr_distance_inp.rTeB);
                assignBlock<16>(wTr, tsr_distance_inp.wTrB);
                assignBlock<6>(bounds.first, tsr_distance_inp.lbB);
                assignBlock<6>(bounds.second, tsr_distance_inp.ubB);


                Eigen::Quaternion<float> q1(eef_pose_w_ref_reference.linear());
                std::array<float, 7> transform1 = {q1.w(), q1.x(), q1.y(), q1.z(), eef_pose_w_ref_reference.translation().x(), eef_pose_w_ref_reference.translation().y(), eef_pose_w_ref_reference.translation().z()};

                Eigen::Quaternion<float> q2(ref_frame_w_world.linear());
                std::array<float, 7> transform2 = {q2.w(), q2.x(), q2.y(), q2.z(), ref_frame_w_world.translation().x(), ref_frame_w_world.translation().y(), ref_frame_w_world.translation().z()};
                assignBlock<7>(transform1, tsr_function_inp.rTeB);
                assignBlock<7>(transform2, tsr_function_inp.wTrB);
                
                assignBlock<6>(bounds.first, tsr_function_inp.lbB);
                assignBlock<6>(bounds.second, tsr_function_inp.ubB);
            }

            // const auto function(const Configuration &q) {
            //     return distanceToConstraint(q);
            // }

            void setComputeFromCenter(bool computeFromCenter) {
                mComputeErrorFromCenter = computeFromCenter;
            }

            bool isComputingFromCenter() const {
                return mComputeErrorFromCenter;
            }

            // void setReferenceFrame(Eigen::Transform<float, 3, Eigen::Isometry> referenceFrame);

            const Eigen::VectorXf distanceToConstraintEEF(Eigen::Transform<float, 3, Eigen::Isometry> computed_eef_pose_world_frame) 
            {
                Eigen::Vector<float, 6> displacement;
                Eigen::Vector<float, 6> penalty;


                const auto actual_pose = world_frame_w_ref_frame * computed_eef_pose_world_frame;
                // std::cout << actual_pose.translation() << std::endl;
                // std::cout << eef_pose_w_ref_reference.translation() << std::endl;

                //Eigen::Vector3
                const auto translation = actual_pose.translation();
                const auto rot_error = actual_pose.linear();

                // const auto translation = actual_pose.translation() - eef_pose_w_ref_reference.translation();
                // const auto rot_error = actual_pose.linear() * eef_pose_w_ref_reference.linear().transpose(); // equivalent to putting transpose on actual_pose

                // const auto translation = computed_eef_pose_world_frame.translation() - eef_pose_w_world_reference.translation();
                // const auto rot_error = computed_eef_pose_world_frame.linear() * eef_pose_w_world_reference.linear().transpose(); // equivalent to putting transpose on actual_pose


                Eigen::AngleAxisf aa(rot_error);
                displacement << translation, aa.axis() * aa.angle() * 0.0;
                // std::cout << displacement << std::endl;


                // (TODO) make this 0 if inside the bounds
                for (size_t i = 0; i < 6; i++)
                {
                    if (displacement[i] < bounds.first[i])
                        penalty[i] = displacement[i] - bounds.first[i];
                    else if (displacement[i] > bounds.second[i])
                        penalty[i] = displacement[i] - bounds.second[i];
                    else
                        penalty[i] = 0.0;
                }

                return penalty;
            }

            const Eigen::VectorXf distanceToConstraint(const ConfigurationArray &q)
            {
                const auto fk = Robot::eefk(q);
                std::cout << fk.matrix() << std::endl;
                return distanceToConstraintEEF(fk);

            }


            // inline statc 


        // const vamp::FloatVector

        // Questions to ask zak
        // How to combine floatvectors
        // how to compare blend operator
        // how to broadcast a single vector as n times -- broadcast_vector

        const auto distanceToConstraintAuto(const ConfigurationBlock &q)
        {
            for (int i=0U; i < 7; i++)
                tsr_function_inp.q[i] = q[i];


            // for (int i=0U; i < 27; i++)
            //     std::cout << tsr_function_inp[i] << " ";
            // std::cout << std::endl;

            // std::cout << tsr_function_inp[0] << "," << tsr_function_inp[10] <<std::endl;

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


        const auto distanceToConstraint(const ConfigurationBlock &q)
        {
            Robot::jacobian_eefk(q, tsr_distance_inp.wTeqB, jac_proj_inp.J);
            // std::cout << "Translation is : " << tsr_distance_inp.wTeqB[12] <<", " << tsr_distance_inp.wTeqB[13] <<", " << tsr_distance_inp.wTeqB[14] << std::endl;
            auto d = tsr_error_simd(tsr_distance_inp);

            return d;

        }

        // TODO(siyer) -- convert this code generatation to a class style
        auto tsr_error_simd(const TSRErrInp &x) noexcept
        {
            vamp::FloatVector<rake, 5> v;
            vamp::FloatVector<rake, 7> y;

            v[0] = (-x[0]) * x[12] + (-x[1]) * x[13] + (-x[2]) * x[14];
            v[1] = (-x[4]) * x[12] + (-x[5]) * x[13] + (-x[6]) * x[14];
            v[2] = (-x[8]) * x[12] + (-x[9]) * x[13] + (-x[10]) * x[14];
            v[3] = x[44] + x[32] * v[0] + x[36] * v[1] + x[40] * v[2];
            v[4] = x[45] + x[33] * v[0] + x[37] * v[1] + x[41] * v[2];
            v[2] = x[46] + x[34] * v[0] + x[38] * v[1] + x[42] * v[2];
            y[0] = (-x[16]) * x[28] + (-x[17]) * x[29] + (-x[18]) * x[30] + x[16] * v[3] + x[17] * v[4] +
                   x[18] * v[2];
            y[1] = (-x[20]) * x[28] + (-x[21]) * x[29] + (-x[22]) * x[30] + x[20] * v[3] + x[21] * v[4] +
                   x[22] * v[2];
            y[2] = (-x[24]) * x[28] + (-x[25]) * x[29] + (-x[26]) * x[30] + x[24] * v[3] + x[25] * v[4] +
                   x[26] * v[2];
            // y[6] = sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]);
            // dependent variables without operations
            y[3] = 0.;
            y[4] = 0.;
            y[5] = 0.;

            // v[0] = x[28] + x[16] * x[12] + x[20] * x[13] + x[24] * x[14];
            // v[1] = x[29] + x[17] * x[12] + x[21] * x[13] + x[25] * x[14];
            // v[2] = x[30] + x[18] * x[12] + x[22] * x[13] + x[26] * x[14];
            // y[0] = (-x[32]) * x[44] + (-x[33]) * x[45] + (-x[34]) * x[46] + x[32] * v[0] + x[33] * v[1] +
            //        x[34] * v[2];
            // y[1] = (-x[36]) * x[44] + (-x[37]) * x[45] + (-x[38]) * x[46] + x[36] * v[0] + x[37] * v[1] +
            //        x[38] * v[2];
            // y[2] = (-x[40]) * x[44] + (-x[41]) * x[45] + (-x[42]) * x[46] + x[40] * v[0] + x[41] * v[1] +
            //        x[42] * v[2];
            // // // dependent variables without operations
            // y[3] = 0.;
            // y[4] = 0.;
            // y[5] = 0.;




            for(int i=0U;i < 6;i++)
            {
                // std::cout << y[i] << " ";
                y[i] = (y[i] - x[48+i]).min(0.F) + (y[i] - x[54+i]).max(0.F);
            }
            // std::cout << std::endl;

            y[6] = (y[0] * y[0] + y[1] * y[1] + y[2] * y[2]).abs();


            for(auto i=0U;i < 6; i++)
                jac_proj_inp.err[i] = y[i];

            // std::cout << "Error vec is : " << jac_proj_inp.err << std::endl;

            return y[6];
        }


        auto jacobian_solve_config(const JacobianProjectInp &x) noexcept
        {

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

        auto jacobian_solve_configCholesky(const JacobianProjectInp &x) noexcept
        {
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
            // q_new = q + grad * 0.5;
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i] + grad[i];
            // std::cout << "q " << q << std::endl;
            // std::cout << "grad " << grad << std::endl;
            // std::cout << "q_new " << q_new << std::endl;
            Robot::descale_configuration_block(q_new);
            q_new = q_new.clamp(0.F, 1.F);
            Robot::scale_configuration_block(q_new);


        }


        auto projectStepJT(const ConfigurationBlock &q, ConfigurationBlock &q_new, bool update_q = true)
        {
            auto dist = distanceToConstraintAuto(q);

            if (update_q)
            {
                auto grad = jacobian_solve_configCholesky(jac_proj_inp);
                // std::cout << "Grad : " << grad << std::endl;
                integrateJointConfiguration(q, q_new, grad);

            }
            return dist;

        }



        bool project(const ConfigurationBlock &q, ConfigurationBlock &q_new)
        {
            bool success = false;
            
            auto dist = distanceToConstraintAuto(q);

            // std::cout << "Initial error is : " << jac_proj_inp.err << std::endl;
            size_t project_iter = 0;
            for (auto i = 0U; i < Robot::dimension; i++)
                q_new[i] = q[i];

            while ((project_iter < 1e3) and (not dist.test_all_less_equal(0.00001F)))
            {
                dist = projectStepJT(q_new, q_new, true);
                // std::cout << "Full error is : " << jac_proj_inp.err << std::endl;
                project_iter += 1;
            }

            if (dist.test_all_less_equal(0.00001F))
                success = true;

            // std::cout << "Num steps : " << project_iter << std::endl;

            // Robot::jacobian_eefk(q_new, tsr_distance_inp.wTeqB, jac_proj_inp.J);
            // std::cout << "FK after proj : " <<  tsr_distance_inp.wTeqB << std::endl;
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