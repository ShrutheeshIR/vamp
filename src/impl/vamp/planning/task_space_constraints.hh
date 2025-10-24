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
            for(auto i=0U; i < 42; i++)
                std::cout << jac_proj_inp.J[{i, 0}] << " ";
            std::cout << std::endl;
            for(auto i=0U; i < 6; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;
            // projectStepJt(q, q_old, true);
            // std::cout << "-------- Printed ------- " << std::endl;

        }

        const auto distanceToConstraint(const ConfigurationBlock &q)
        {
            for (int i=0U; i < Robot::dimension; i++)
                tsr_function_inp.q[i] = q[i];
            
            // Robot::tsr_function_ori_blend_simd(tsr_function_inp, jac_proj_inp);
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

        auto projectStepJt(const ConfigurationBlock &q, ConfigurationBlock &q_new, bool update_q = true)
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "dist base " << dist << std::endl;
            ConfigurationBlock grad;
            if (update_q)
            {
                // auto grad = jacobian_solve_config_jt(jac_proj_inp);
                Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
                // for(auto i=0U; i < 7; i++)
                //     std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;


                integrateJointConfigurationJt(q, q_new, grad);
            }
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

            // std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;

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