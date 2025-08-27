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
            const size_t max_project_iters = 1000;

        public:
            void integrateJointConfiguration(const ConfigurationArray &q, ConfigurationArray &q_new, const Eigen::Matrix<float, Eigen::Dynamic, 1> &update_vector)
            {
                for (size_t i = 0; i < q.size(); i ++)
                    // need to adjust joint limits
                    q_new[i] = q[i] + update_vector(i);
            }

            double projectStepOld(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
            {
                ;
                // Eigen::Matrix<float, 6, Eigen::Dynamic> J;
                // Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

                // Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
                // const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

                // if (update_q)
                // {
                //     // need to convert jacobian into ref frame
                //     const Eigen::Matrix<float, Eigen::Dynamic, 1> update = -J.transpose() * (J * J.transpose() + 1e-6 * Eigen::Matrix<float, 6, 6>::Identity()).ldlt().solve(distance);
                //     integrateJointConfiguration(q, q_new, update);
                // }
                // return distance.squaredNorm();

            }


            double projectStepJT(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
            {
                // Eigen::Matrix<float, 6, Eigen::Dynamic> J;
                // Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

                // Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
                // const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

                // if (update_q)
                // {
                //     // need to convert jacobian into ref frame
                //     const Eigen::Matrix<float, Eigen::Dynamic, 1> update = -J.transpose() * distance;
                //     integrateJointConfiguration(q, q_new, update);
                // }
                // return distance.squaredNorm();

            }

            // project with vamp inverse
            double projectStep(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
            {
                // Eigen::Matrix<float, 6, Eigen::Dynamic> J;
                // Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;
                // Eigen::Matrix<float, 7, 1> grad;

                // Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
                // const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

                // std::array<float, 6> dist_arr;
                // std::copy(distance.data(), distance.data() + distance.size(), dist_arr.begin());



                // if (update_q)
                // {
                //     // ;
                    
                //     Robot::jacobian_solve(q, dist_arr, grad);
                //     // std::cout << grad << std::endl;
                //     integrateJointConfiguration(q, q_new, -grad);
                // }
                // return distance.squaredNorm();
                ;

            }


            bool project(const ConfigurationArray &q, ConfigurationArray &q_new)
            {
                // double distance = projectStep(q, q_new, false);
                // size_t project_iter = 0;
                // bool success = false;
                // // std::cout << distance << " " << std::endl;
                // while ((project_iter < max_project_iters) && (distance > tolerance))
                // {
                //     distance = projectStepJT(q_new, q_new, true);
                //     // std::cout << distance << " " << std::endl;
                // }
                // if (distance < tolerance)
                // {
                //     // std::cout << distance << " " << std::endl;
                //     const auto fk = Robot::eefk(q_new);
                //     std::cout << fk.matrix()(2,3) << std::endl;

                //     success = true;
                // }
                // return success;
                ;
                    
            }

            bool isValid(const ConfigurationArray &q)
            {
                const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose = Robot::eefk(q);
                const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

                return distance.squaredNorm() < tolerance;
            }

            virtual const Eigen::VectorXf distanceToConstraintEEF(Eigen::Transform<float, 3, Eigen::Isometry> computed_eef_pose_world_frame) = 0;
            
    };

    // We will work with the assumption that all the transforms are given to us
    template <typename Robot, std::size_t rake>
    class TaskSpaceConstraint : public RobotConstraint<Robot>
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;

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
            // vamp::FloatVector<16>rTe;
            std::array<float, 16> rTe;
            vamp::FloatVector<rake, 16>rTeB;

            // vamp::FloatVector<16>wTr;
            std::array<float, 16> wTr;
            vamp::FloatVector<rake, 16>wTrB;

            bool mComputeErrorFromCenter;




            struct TSRErrInp {
                vamp::FloatVector<rake, 16>rTeB;
                vamp::FloatVector<rake, 16> wTrB;
                vamp::FloatVector<rake, 16> wTeqB;
                vamp::FloatVector<rake, 6> lbB;
                vamp::FloatVector<rake, 6> ubB;
                
                static inline auto operator[](size_t index) const {
                    if (index >= 0 && index < 16)
                        return rTeB[index];
                    else if (index >= 16 && index < 32)
                        return wTrB[index - 16];
                    else if (index >= 32 && index < 48)
                        return wTeqB[index - 32];
                    else if (index >= 48 && index < 54)
                        return wTrB[index - 48];
                    else if (index >= 54 && index < 60)
                        return wTrB[index - 54];
                }
                    

                    // Add bounds checking for robustness
                    // if (index >= indices.back()) {
                    //     throw std::out_of_range("Index out of bounds");
                    // }

                    // for (size_t i = 0; i < container.size() - 1; i++)
                    //     if (index >= indices[i] && index < indices[i+1])
                    //         return container[i];

                }


            };


        public:

            TaskSpaceConstraint(
                const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference,
                const Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world,
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

                TSRErrInp tsr_distance_inp;
                tsr_distance_inp.rTeB = Vector(rTe, true);
                tsr_distance_inp.wTrB = Vector(wTr, true);

                tsr_distance_inp.lbB = Vector(bounds.first(), true);
                tsr_distance_inp.ubB = Vector(bounds.second(), true);

            }

            const auto function(const Configuration &q) {
                return distanceToConstraint(q);
            }

            void setComputeFromCenter(bool computeFromCenter) {
                mComputeErrorFromCenter = computeFromCenter;
            }

            bool isComputingFromCenter() const {
                return mComputeErrorFromCenter;
            }

            // void setReferenceFrame(Eigen::Transform<float, 3, Eigen::Isometry> referenceFrame);

            const Eigen::VectorXf distanceToConstraintEEF(Eigen::Transform<float, 3, Eigen::Isometry> computed_eef_pose_world_frame) override
            {
                Eigen::Vector<float, 6> displacement;
                Eigen::Vector<float, 6> penalty;


                // const auto actual_pose = world_frame_w_ref_frame * computed_eef_pose_world_frame;

                //Eigen::Vector3
                // const auto translation = actual_pose.translation() - eef_pose_w_ref_reference.translation();
                // const auto rot_error = actual_pose.linear() * eef_pose_w_ref_reference.linear().transpose(); // equivalent to putting transpose on actual_pose


                const auto translation = computed_eef_pose_world_frame.translation() - eef_pose_w_world_reference.translation();
                const auto rot_error = computed_eef_pose_world_frame.linear() * eef_pose_w_world_reference.linear().transpose(); // equivalent to putting transpose on actual_pose


                Eigen::AngleAxisf aa(rot_error);

                // Eigen::Vec6
                displacement << aa.axis() * aa.angle(), translation;


                // (TODO) make this 0 if inside the bounds
                // for (size_t i = 0; i < 6; i++)
                // {
                //     if (displacement[i] < bounds.first(i))
                //         penalty[i] = displacement[i] - bounds.first(i);
                //     else if (displacement[i] > bounds.second(i))
                //         penalty[i] = displacement[i] - bounds.second(i);
                //     else
                //         penalty[i] = 0.0;
                // }

                return penalty;
            }

            const Eigen::VectorXf distanceToConstraint(const ConfigurationArray &q)
            {
                const auto fk = Robot::eefk(q);
                return distanceToConstraintEEF(fk);

            }


            // inline statc 


        // const vamp::FloatVector

        // Questions to ask zak
        // How to combine floatvectors
        // how to compare blend operator
        // how to broadcast a single vector as n times -- broadcast_vector

        // TODO(siyer) -- convert this code generatation to a class style
        inline static auto tsr_error_simd(const vamp::FloatVector<rake, 60> &x) noexcept
        {
            vamp::FloatVector<rake, 7> v;
            vamp::FloatVector<rake, 7> y;

            v[0] = (-x[0]) * x[12] + (-x[1]) * x[13] + (-x[2]) * x[14];
            v[1] = (-x[4]) * x[12] + (-x[5]) * x[13] + (-x[6]) * x[14];
            v[2] = (-x[8]) * x[12] + (-x[9]) * x[13] + (-x[10]) * x[14];
            v[3] = x[44] + x[32] * v[0] + x[36] * v[1] + x[40] * v[2];
            v[4] = x[45] + x[33] * v[0] + x[37] * v[1] + x[41] * v[2];
            v[2] = x[46] + x[34] * v[0] + x[38] * v[1] + x[42] * v[2];
            v[1] = (-x[16]) * x[28] + (-x[17]) * x[29] + (-x[18]) * x[30] + x[16] * v[3] + x[17] * v[4] +
                   x[18] * v[2];
            v[0] = v[1] - x[48];
            v[0] = min(v[0], 0.F);
            v[5] = v[1] - x[54];
            v[5] = max(v[5], 0.F);
            y[0] = v[0] + v[5];
            v[5] = (-x[20]) * x[28] + (-x[21]) * x[29] + (-x[22]) * x[30] + x[20] * v[3] + x[21] * v[4] +
                   x[22] * v[2];
            v[0] = v[5] - x[49];
            v[0] = min(v[0], 0.F);
            v[6] = v[5] - x[55];
            v[6] = max(v[6], 0.F);
            y[1] = v[0] + v[6];
            v[2] = (-x[24]) * x[28] + (-x[25]) * x[29] + (-x[26]) * x[30] + x[24] * v[3] + x[25] * v[4] +
                   x[26] * v[2];
            v[4] = v[2] - x[50];
            v[4] = min(v[4], 0.F);
            v[3] = v[2] - x[56];
            v[3] = max(v[3], 0.F);
            y[2] = v[4] + v[3];
            v[3] = 0. - x[51];
            v[3] = min(v[3], 0.F);
            v[4] = 0. - x[57];
            v[4] = max(v[4], 0.F);
            y[3] = v[3] + v[4];
            v[4] = 0. - x[52];
            v[4] = min(v[4], 0.F);
            v[3] = 0. - x[58];
            v[3] = max(v[3], 0.F);
            y[4] = v[4] + v[3];
            v[3] = 0. - x[53];
            v[3] = min(v[3], 0.F);
            v[4] = 0. - x[59];
            v[4] = max(v[4], 0.F);
            y[5] = v[3] + v[4];
            y[6] = std::sqrt(v[1] * v[1] + v[5] * v[5] + v[2] * v[2]);
        }



        // inline static auto tsr_error_simd(const std::array<float, 60> &x) noexcept
        // {
        //     std::array<float, 7> v;
        //     std::array<float, 7> y;

        //     v[0] = (-x[0]) * x[12] + (-x[1]) * x[13] + (-x[2]) * x[14];
        //     v[1] = (-x[4]) * x[12] + (-x[5]) * x[13] + (-x[6]) * x[14];
        //     v[2] = (-x[8]) * x[12] + (-x[9]) * x[13] + (-x[10]) * x[14];
        //     v[3] = x[44] + x[32] * v[0] + x[36] * v[1] + x[40] * v[2];
        //     v[4] = x[45] + x[33] * v[0] + x[37] * v[1] + x[41] * v[2];
        //     v[2] = x[46] + x[34] * v[0] + x[38] * v[1] + x[42] * v[2];
        //     v[1] = (-x[16]) * x[28] + (-x[17]) * x[29] + (-x[18]) * x[30] + x[16] * v[3] + x[17] * v[4] +
        //            x[18] * v[2];
        //     v[0] = v[1] - x[48];
        //     v[0] = std::min(v[0], 0.);
        //     // if (0. < v[0])
        //     // {
        //     //     v[0] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[0] = v[0];
        //     // }
        //     v[5] = v[1] - x[54];
        //     v[5] = std::max(v[5], 0.F);
        //     // if (v[5] < 0.)
        //     // {
        //     //     v[5] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[5] = v[5];
        //     // }
        //     y[0] = v[0] + v[5];
        //     v[5] = (-x[20]) * x[28] + (-x[21]) * x[29] + (-x[22]) * x[30] + x[20] * v[3] + x[21] * v[4] +
        //            x[22] * v[2];
        //     v[0] = v[5] - x[49];
        //     v[0] = std::min(v[0], 0.F);
        //     // if (0. < v[0])
        //     // {
        //     //     v[0] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[0] = v[0];
        //     // }
        //     v[6] = v[5] - x[55];
        //     v[6] = std::max(v[6], 0.F);
        //     // if (v[6] < 0.)
        //     // {
        //     //     v[6] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[6] = v[6];
        //     // }
        //     y[1] = v[0] + v[6];
        //     v[2] = (-x[24]) * x[28] + (-x[25]) * x[29] + (-x[26]) * x[30] + x[24] * v[3] + x[25] * v[4] +
        //            x[26] * v[2];
        //     v[4] = v[2] - x[50];
        //     v[4] = std::min(v[4], 0.F);
        //     // if (0. < v[4])
        //     // {
        //     //     v[4] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[4] = v[4];
        //     // }
        //     v[3] = v[2] - x[56];
        //     v[3] = std::max(v[3], 0.F);
        //     // if (v[3] < 0.)
        //     // {
        //     //     v[3] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[3] = v[3];
        //     // }
        //     y[2] = v[4] + v[3];
        //     v[3] = 0. - x[51];
        //     v[3] = std::min(v[3], 0.F);
        //     // if (0. < v[3])
        //     // {
        //     //     v[3] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[3] = v[3];
        //     // }
        //     v[4] = 0. - x[57];
        //     v[4] = max(v[4], 0.F);
        //     // if (v[4] < 0.)
        //     // {
        //     //     v[4] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[4] = v[4];
        //     // }
        //     y[3] = v[3] + v[4];
        //     v[4] = 0. - x[52];
        //     v[4] = std::min(v[4], 0.F);
        //     // if (0. < v[4])
        //     // {
        //     //     v[4] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[4] = v[4];
        //     // }
        //     v[3] = 0. - x[58];
        //     v[3] = std::max(v[3], 0.F);
        //     // if (v[3] < 0.)
        //     // {
        //     //     v[3] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[3] = v[3];
        //     // }
        //     y[4] = v[4] + v[3];
        //     v[3] = 0. - x[53];
        //     v[3] = std::min(v[3], 0.F);
        //     // if (0. < v[3])
        //     // {
        //     //     v[3] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[3] = v[3];
        //     // }
        //     v[4] = 0. - x[59];
        //     v[4] = std::max(v[4], 0.F);
        //     // if (v[4] < 0.)
        //     // {
        //     //     v[4] = 0.;
        //     // }
        //     // else
        //     // {
        //     //     v[4] = v[4];
        //     // }
        //     y[5] = v[3] + v[4];
        //     y[6] = std::sqrt(v[1] * v[1] + v[5] * v[5] + v[2] * v[2]);
        // }



    };

}