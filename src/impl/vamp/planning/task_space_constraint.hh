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
        inline static void assignBlock(std::array<float, dim> src, vamp::FloatVector<rake, dim> &dest)
        {
            for (size_t i = 0; i < dim; i++)
            {
                dest[i] = src[i];
            }
        }


    public:
        // virtual vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) = 0;

        inline void integrateJointConfiguration(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            const ConfigurationBlock &gradient,
            float alpha = 1.0F)
        {
            // ConfigurationBlock q_new;
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                q_new[i] = q[i] - gradient[i] * alpha;
            }
            Robot::descale_configuration_block(q_new);
            q_new = q_new.clamp(0.F, 1.F);
            Robot::scale_configuration_block(q_new);
            // return q_new;
        }

        // virtual vamp::FloatVector<rake, 1> projectStep(
        //     const ConfigurationBlock &q,
        //     ConfigurationBlock &q_new,
        //     ProjMethod projection_method = ProjMethod::InnerLM,
        //     bool update_q = true) = 0;

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

            void print() const
            {
                std::cout << "rTlB: ";
                for (int i = 0U; i < 7; i ++)
                    std::cout << rTlB[{i, 0}] << " ";
                std::cout << std::endl;

                std::cout << "lbB: ";
                for (int i = 0U; i < 6; i ++)
                    std::cout << lbB[{i, 0}] << " ";
                std::cout << std::endl;

                std::cout << "ubB: ";
                for (int i = 0U; i < 6; i ++)
                    std::cout << ubB[{i, 0}] << " ";
                std::cout << std::endl;
            }
        };

        mutable TSRComputeInput tsr_function_inp;

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

        mutable JacobianProjectInp jac_proj_inp;
        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        static constexpr char* name = "B";
        size_t num_project_step_called = 0;
        BimanualTaskSpaceConstraint(
            const std::array<float, 7> right_eef_pose_w_ref_left_eef,  // rTl qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6> lower_bound,
            const std::array<float, 6> upper_bound
    )
        {
            // Eigen::Quaternion<float> q1(right_eef_pose_w_ref_left_eef.linear());
            // std::array<float, 7> transform1 = {
            //     q1.w(),
            //     q1.x(),
            //     q1.y(),
            //     q1.z(),
            //     right_eef_pose_w_ref_left_eef.translation().x(),
            //     right_eef_pose_w_ref_left_eef.translation().y(),
            //     right_eef_pose_w_ref_left_eef.translation().z()};

            RobotConstraint<Robot, rake>::template assignBlock<7>(right_eef_pose_w_ref_left_eef, tsr_function_inp.rTlB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(lower_bound, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6>(upper_bound, tsr_function_inp.ubB);
            // tsr_function_inp.print();
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            // for(auto i=0U; i < Robot::dimension + 19; i++)
            //     std::cout << tsr_function_inp[i] << " ";


            auto dist = distanceToConstraint(q);
            std::cout << "Bimanual Error : " << std::endl;
            for(auto i=0U; i < 6 * Robot::dimension; i++){
                if (i%Robot::dimension == 0)
                    std::cout << std::endl << " J[" << i << "]: ";
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl << "Error : ";
            for(auto i=0U; i < 6; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;
            return dist;
        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {
            for (size_t i = 0; i < Robot::dimension; i++)
            {
                tsr_function_inp.q[i] = q[i];
            }

            // std::cout << "Q input: ";
            // for(auto i=0U; i < Robot::dimension; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.q[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.rTlB[{i, 0}] << " ";
            // std::cout << std::endl;


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
            // std::cout << "Bimanual Error : ";
            // for(auto i=0U; i < 6; i++){
            //     std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            // }
            // for(auto i=0U; i < 6 * Robot::dimension; i++){
            //     if (i%Robot::dimension == 0)
            //         std::cout << std::endl << " J[" << i << "]: ";
            //     std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            // }

            // std::cout << std::endl;

            return d;
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            bool update_q = true,
            float alpha = 1.0F)
        {

            auto dist = distanceToConstraint(q);
            // std::cout << "Bimanual constraint distance: " << dist << std::endl;
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_tsr_relative_error_lm_inner<rake>(jac_proj_inp, grad);
                // std::cout << "Grad for bimanual constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;
                // grad = grad.zero_out_nans();
                // std::cout << "Grad for bimanual constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;

            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_tsr_relative_error_lm_outer<rake>(jac_proj_inp, grad);
                // std::cout << "Grad for bimanual constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;
            }
            else if (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_tsr_relative_error_gradient_descent<rake>(jac_proj_inp, grad);
                // std::cout << "Grad for bimanual constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;
            }
            else {
                std::cout << "Invalid projection method: " << projection_method << std::endl;
                throw std::runtime_error("Invalid projection method");
            }
            ConfigurationBlock q_new;
            RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            return q_new;
        }


    };

    template <typename Robot, std::size_t rake>
    class SelfCollisionConstraint : public RobotConstraint<Robot, rake>
    {
        /**
         */
    protected:
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

        struct JacobianProjectInp
        {
            vamp::FloatVector<rake, Robot::num_bounding_spheres * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, Robot::num_bounding_spheres> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < Robot::num_bounding_spheres * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_bounding_spheres *  Robot::dimension &&
                    index < Robot::num_bounding_spheres * Robot::dimension + 6 * Robot::n_eef)
                {
                    return err[index - Robot::num_bounding_spheres * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < Robot::num_bounding_spheres * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= Robot::num_bounding_spheres * Robot::dimension &&
                    index < Robot::num_bounding_spheres * Robot::dimension + Robot::num_bounding_spheres * Robot::n_eef)
                {
                    return err[index - Robot::num_bounding_spheres * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, Robot::num_bounding_spheres * Robot::n_eef + Robot::num_bounding_spheres * Robot::n_eef * Robot::dimension> y)
            {
                for (size_t i = 0; i < Robot::num_bounding_spheres; i++)
                {
                    err[i] = y[Robot::num_bounding_spheres * Robot::dimension + i];
                }
                for (size_t i = 0; i < Robot::num_bounding_spheres * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        mutable JacobianProjectInp jac_proj_inp;
        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        static constexpr char* name = "SelfCollisionConstraint";
        SelfCollisionConstraint()
        {
            ;
        }

        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            // for(auto i=0U; i < Robot::dimension + 19; i++)
            //     std::cout << tsr_function_inp[i] << " ";


            auto dist = distanceToConstraint(q);
            std::cout << "Self collision error : " << std::endl;
            for(auto i=0U; i < Robot::num_bounding_spheres * Robot::dimension; i++){
                if (i%Robot::dimension == 0)
                    std::cout << std::endl << " J[" << i << "]: ";
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl << "Error : ";
            for(auto i=0U; i < Robot::num_bounding_spheres; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;
            return dist;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {

            Robot::template bounding_spheres_self_collision_error<rake>(q, jac_proj_inp);


            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < Robot::num_bounding_spheres; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }
            // std::cout << "Error : ";
            // for(auto i=0U; i < Robot::num_bounding_spheres; i++){
            //     std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            // }
            // std::cout << std::endl;

            return d * 0.0;
        }

        vamp::FloatVector<rake, 1> projectStep(
            const ConfigurationBlock &q,
            ConfigurationBlock &q_new,
            ProjMethod projection_method = ProjMethod::InnerLM,
            bool update_q = true,
            float alpha = 1.0F)
        {
            auto dist = print_robot_tsr_error(q);
            if (update_q)
            {
                ConfigurationBlock grad;

                if (projection_method == ProjMethod::InnerLM)
                {
                    Robot::template solve_self_collision_error_lm_inner<rake>(jac_proj_inp, grad);
                    // grad = grad.zero_out_nans();
                    // std::cout << "Grad for selfcoll constraint: "  ;
                    // for (auto i = 0U; i < Robot::dimension; i++)
                    //         std::cout << q[{i, 0}] << " -- " <<grad[{i, 0}] << " ";
                    // std::cout << std::endl;
                }
                if (projection_method == ProjMethod::OuterLM)
                {
                    Robot::template solve_self_collision_error_lm_outer<rake>(jac_proj_inp, grad);
                }
                if (projection_method == ProjMethod::GradDesc)
                {
                    Robot::template solve_self_collision_error_gradient_descent<rake>(jac_proj_inp, grad);
                }
                RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            }
            return dist;
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

        mutable TSRComputeInput tsr_function_inp;

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

        mutable JacobianProjectInp jac_proj_inp;
        // some housekeeping variables predefined for speed

        ConfigurationBlock q_old;

    public:
        static constexpr char* name = "T";
        size_t num_project_step_called = 0;
        TaskSpaceConstraint(
            std::array<std::array<float, 7>, Robot::n_eef> eef_pose_w_ref_reference, // qw, qx, qy, qz, tx, ty, tz
            std::array<std::array<float, 7>, Robot::n_eef> ref_frame_w_world, // qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6 * Robot::n_eef> lower_bound,
            const std::array<float, 6 * Robot::n_eef> upper_bound
        )
        {


            std::array<float, 7 * Robot::n_eef> transform1;
            std::memcpy(transform1.data(), eef_pose_w_ref_reference.data(), sizeof(float) * 7 * Robot::n_eef);

            std::array<float, 7 * Robot::n_eef> transform2;
            std::memcpy(transform2.data(), ref_frame_w_world.data(), sizeof(float) * 7 * Robot::n_eef);

            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform1, tsr_function_inp.rTeB);
            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform2, tsr_function_inp.wTrB);

            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(lower_bound, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(upper_bound, tsr_function_inp.ubB);
        }


        vamp::FloatVector<rake, 1> print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "Q input: ";
            // for(auto i=0U; i < Robot::dimension; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.q[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.rTeB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.wTrB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Lower limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.lbB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Upper limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.ubB[{i, 0}] << " ";
            // std::cout << std::endl;


            std::cout << "TSR Error : " << std::endl;
            for(auto i=0U; i < 6 * Robot::n_eef * Robot::dimension; i++){
                if(i % Robot::dimension == 0)
                    std::cout << std::endl << "J[" << i / Robot::dimension << "] : ";
                std::cout << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            std::cout << "TSR Error : ";
            for(auto i=0U; i < 6 * Robot::n_eef; i++)
                std::cout << jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;

            return dist;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
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



            // for(int i = 0U; i < 6 * 2; i++)
            //     std::cout << short_jac_proj_inp.err[{i, 0}] << ", ";
            // std::cout << std::endl;


            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0];
            for (size_t i = 1; i < 6 * Robot::n_eef; i++)
            {
                d = d + jac_proj_inp.err[i] * jac_proj_inp.err[i];
            }

            // for(int i = 0U; i < 6 * Robot::n_eef; i++)
            //     std::cout << jac_proj_inp.err[{i, 0}] << ", ";
            // std::cout << std::endl;

            // std::cout << "TSR Error : ";
            // for(auto i=0U; i < 6 * 2; i++)
            //     std::cout << short_jac_proj_inp.err[{i, 0}] << " ";
            // std::cout << std::endl;

            return d;
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            auto dist = distanceToConstraint(q);
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                // Robot::template solve_2_eef_tsr_error_lm_inner<rake>(short_jac_proj_inp, grad);
                // grad = grad.zero_out_nans();

                Robot::template solve_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
                // std::cout << "Grad for TSR constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;
            }
            else if (projection_method == ProjMethod::OuterLM)
            {
                // Robot::template solve_2_eef_tsr_error_lm_outer<rake>(short_jac_proj_inp, grad);
                Robot::template solve_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if  (projection_method == ProjMethod::GradDesc)
            {
                // Robot::template solve_2_eef_tsr_error_gradient_descent<rake>(short_jac_proj_inp, grad);
                Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else {
                throw std::runtime_error("Invalid projection method");
            }
            ConfigurationBlock q_new;

            RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            return q_new;
        }


    };

    template <typename Robot, std::size_t rake>
    class FeetTaskSpaceConstraint : public RobotConstraint<Robot, rake>
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

        mutable TSRComputeInput tsr_function_inp;

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

        mutable JacobianProjectInp jac_proj_inp;
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

        mutable ShortenedJacobianProjectInp short_jac_proj_inp;


        ConfigurationBlock q_old;

    public:
        static constexpr char* name = "F";
        size_t num_project_step_called = 0;

        FeetTaskSpaceConstraint(
            std::array<std::array<float, 7>, Robot::n_eef> eef_pose_w_ref_reference, // qw, qx, qy, qz, tx, ty, tz
            std::array<std::array<float, 7>, Robot::n_eef> ref_frame_w_world, // qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6 * Robot::n_eef> lower_bound,
            const std::array<float, 6 * Robot::n_eef> upper_bound
        )
        {


            std::array<float, 7 * Robot::n_eef> transform1;
            std::memcpy(transform1.data(), eef_pose_w_ref_reference.data(), sizeof(float) * 7 * Robot::n_eef);

            std::array<float, 7 * Robot::n_eef> transform2;
            std::memcpy(transform2.data(), ref_frame_w_world.data(), sizeof(float) * 7 * Robot::n_eef);

            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform1, tsr_function_inp.rTeB);
            RobotConstraint<Robot, rake>::template assignBlock<7 * Robot::n_eef>(transform2, tsr_function_inp.wTrB);

            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(lower_bound, tsr_function_inp.lbB);
            RobotConstraint<Robot, rake>::template assignBlock<6 * Robot::n_eef>(upper_bound, tsr_function_inp.ubB);
        }


        auto print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "Q input: ";
            // for(auto i=0U; i < Robot::dimension; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.q[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.rTeB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Transform input1: ";
            // for(auto i=0U; i < 7; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.wTrB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Lower limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.lbB[{i, 0}] << " ";
            // std::cout << std::endl;
            // std::cout << "Upper limit : ";
            // for(auto i=0U; i < 6; i++)
            //     std::cout << std::setprecision(5) << tsr_function_inp.ubB[{i, 0}] << " ";
            // std::cout << std::endl;


            std::cout << "TSR Error : " << std::endl;
            for(auto i=0U; i < 6 * 2 * Robot::dimension; i++){
                if(i % Robot::dimension == 0)
                    std::cout << std::endl << "J[" << i / Robot::dimension << "] : ";
                std::cout << short_jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            std::cout << "Error : " << std::endl;
            for(auto i=0U; i < 6 * 2; i++)
                std::cout << short_jac_proj_inp.err[{i, 0}] << " ";
            std::cout << std::endl;


            return dist;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
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


            auto d = short_jac_proj_inp.err[0] * short_jac_proj_inp.err[0];
            for (size_t i = 1; i < 6 * 2; i++)
            {
                d = d + short_jac_proj_inp.err[i] * short_jac_proj_inp.err[i];
            }
            // std::cout << "TSR Error : ";
            // for(auto i=0U; i < 6 * 2; i++)
            //     std::cout << short_jac_proj_inp.err[{i, 0}] << " ";
            // std::cout << std::endl;

            return d;
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            auto dist = distanceToConstraint(q);
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_2_eef_tsr_error_lm_inner<rake>(short_jac_proj_inp, grad);
                // grad = grad.zero_out_nans();
                // std::cout << "Grad for TSR constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << q[{i, 0}] << " -- " <<grad[{i, 0}] << " ";
                // std::cout << std::endl;

                // Robot::template solve_tsr_error_lm_inner<rake>(jac_proj_inp, grad);
            }
            else if  (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_2_eef_tsr_error_lm_outer<rake>(short_jac_proj_inp, grad);
                // Robot::template solve_tsr_error_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if  (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_2_eef_tsr_error_gradient_descent<rake>(short_jac_proj_inp, grad);
                // Robot::template solve_tsr_error_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else {
                throw std::runtime_error("Invalid projection method");
            }
            ConfigurationBlock q_new;
            RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            return q_new;
        }


    };

    template <typename Robot, std::size_t rake, std::size_t num_polygons>
    class CoMTaskSpaceConstraint : public RobotConstraint<Robot, rake>
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
            vamp::FloatVector<rake, 2 * Robot::dimension> J;  // jacobian
            vamp::FloatVector<rake, 2> err;                   // error vector

            auto &operator[](size_t index)
            {
                if (index < 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 2 *  Robot::dimension &&
                    index < 2 * Robot::dimension + 2)
                {
                    return err[index - 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            const auto operator[](size_t index) const
            {
                if (index < 2 * Robot::dimension)
                {
                    return J[index];
                }
                else if (
                    index >= 2 * Robot::dimension &&
                    index < 2 * Robot::dimension + 2)
                {
                    return err[index - 2 * Robot::dimension];
                }
                else
                {
                    return err[0];
                }
            }

            JacobianProjectInp &
            operator=(vamp::FloatVector<rake, 2 + 2 * Robot::dimension> y)
            {
                for (size_t i = 0; i < 2; i++)
                {
                    err[i] = y[2 * Robot::dimension + i];
                }
                for (size_t i = 0; i < 2 * Robot::dimension; i++)
                {
                    J[i] = y[i];
                }
                return *this;
            }
        };

        mutable JacobianProjectInp jac_proj_inp;
        mutable CoMConstraintInp com_jac_polygons;

        // size_t num_polygons;

        // some housekeeping variables predefined for speed
        ConfigurationBlock q_old;

    public:
        static constexpr char* name = "C";
        size_t num_project_step_called = 0;
        CoMTaskSpaceConstraint(
            const std::array<float, 2 * num_polygons> polygon_points)
        {
            // for (size_t i = 0; i < 2 * num_polygons; i++)
            //     std::cout << polygon_points[i] << " ";
            // std::cout << std::endl;


            RobotConstraint<Robot, rake>::template assignBlock<2 * num_polygons>(polygon_points, com_jac_polygons.polygon_points);
        }
        auto print_robot_tsr_error(const ConfigurationBlock &q) const
        {
            auto dist = distanceToConstraint(q);
            std::cout << "COM  : ";
            for(auto i=0U; i < 3; i++){
                std::cout << std::setprecision(5) << com_jac_polygons.CoM[{i, 0}] << " ";
            }
            std::cout << std::endl;

            std::cout << "COM Error : ";
            for(auto i=0U; i < 2; i++){
                std::cout << std::setprecision(5) << jac_proj_inp.err[{i, 0}] << " ";
            }
            std::cout << std::endl;
            std::cout << "Error Jac : ";
            for(auto i=0U; i < 2 * Robot::dimension; i++){
                if (i%Robot::dimension == 0)
                    std::cout << std::endl;
                std::cout << std::setprecision(5) << jac_proj_inp.J[{i, 0}] << " ";
            }
            std::cout << std::endl;
            return dist;

        }

        vamp::FloatVector<rake, 1> distanceToConstraint(const ConfigurationBlock &q) const
        {

            Robot::template compute_com<rake>(q, com_jac_polygons);
            // com_jac_polygons.com must be offset by the first floating joint
            // for(auto i=0U; i < 3; i++){
            //     com_jac_polygons.CoM[i] = com_jac_polygons.CoM[i] - q[i];
            // }


            Robot::template com_constraint_error<rake>(com_jac_polygons, num_polygons, jac_proj_inp);
            // std::cout << "COM  : ";
            // for(auto i=0U; i < 3; i++){
            //     std::cout << std::setprecision(5) << com_jac_polygons.CoM[{i, 0}] << " ";
            // }

            // std::cout << "COM Error : ";
            // for(auto i=0U; i < 2; i++){
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




            auto d = jac_proj_inp.err[0] * jac_proj_inp.err[0] + jac_proj_inp.err[1] * jac_proj_inp.err[1];

            return d;
        }

        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0F)
        {
            auto dist = distanceToConstraint(q);
            // std::cout << "COM constraint distance: " << dist << std::endl;
            typename Robot::template ConfigurationBlock<rake> grad;

            if (projection_method == ProjMethod::InnerLM)
            {
                Robot::template solve_com_function_lm_inner<rake>(jac_proj_inp, grad);
                // grad = grad.zero_out_nans();
                // std::cout << "Grad for COM constraint: "  ;
                // for (auto i = 0U; i < Robot::dimension; i++)
                //         std::cout << grad[{i, 0}] << " ";
                // std::cout << std::endl;

            }
            else if  (projection_method == ProjMethod::OuterLM)
            {
                Robot::template solve_com_function_lm_outer<rake>(jac_proj_inp, grad);
            }
            else if  (projection_method == ProjMethod::GradDesc)
            {
                Robot::template solve_com_function_gradient_descent<rake>(jac_proj_inp, grad);
            }
            else {
                throw std::invalid_argument("Invalid projection method");
            }
            ConfigurationBlock q_new;
            RobotConstraint<Robot, rake>::integrateJointConfiguration(q, q_new, grad, alpha);
            return q_new;
        }

    };



    template <typename Robot, std::size_t rake, typename... Constraints>
    class ComposableConstraints : public RobotConstraint<Robot, rake>
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

        // vamp::FloatVector<rake, 1> projectStep(
        //     const ConfigurationBlock &q,
        //     ConfigurationBlock &q_new,
        //     ProjMethod projection_method = ProjMethod::InnerLM,
        //     float alpha = 1.0f)
        // {
        //     ConfigurationBlock q_in = q;

        //     std::apply([&](auto&... c) {
        //         ((c.projectStep(q_in, q_new, projection_method, alpha), q_in = q_new), ...);
        //     }, constraints_);

        //     return distanceToConstraint(q_new);
        // }
        ConfigurationBlock projectStep(
            const ConfigurationBlock &q,
            ProjMethod projection_method = ProjMethod::InnerLM,
            float alpha = 1.0f)
        {
            // Copy initial configuration
            ConfigurationBlock q_in;
            ConfigurationBlock q_new;
            for(size_t dim = 0U; dim < Robot::dimension; dim++) {
                q_in[dim] = q[dim];
                q_new[dim] = q[dim];
            }

            // Lambda to process a single constraint
            auto applyConstraint = [&](auto& c) {
                // Optional debug print
                // std::cout << c.name << std::endl;
                // asm volatile("" ::: "memory");
                // Call the projection step
                q_new = c.projectStep(q_in, projection_method, alpha);
                c.num_project_step_called++;
                // std::cout << q_new << std::endl;

                // Update q_in to the latest projected configuration
                for(size_t dim = 0U; dim < Robot::dimension; dim++) {
                    q_in[dim] = q_new[dim];
                }

                // Optional debug print
                // std::cout << "After projectStep for constraint: " << c.name << std::endl;
            };

            // Apply the lambda to all constraints in order
            std::apply([&](auto&... cs) {
                // Left-to-right evaluation guaranteed
                (applyConstraint(cs), ...);
            }, constraints_);

            // Return distance to constraints as before
            return q_new;
        }

        void printNumProjectStepCalled() const {
            std::apply([&](auto&... cs) {
                ((std::cout << cs.name << " "
                            << cs.num_project_step_called << " "), ...);
            }, constraints_);

            std::cout << '\n';
        }

        void resetNumProjectStepCalled() {
            std::apply([&](auto&... cs) {
                ((cs.num_project_step_called = 0), ...);
            }, constraints_);
        }

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
                    q_new = projectStep(q_old, projection_method, descend_rate);
                    dist = distanceToConstraint(q_new);
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
                    printNumProjectStepCalled();
                    resetNumProjectStepCalled();
                    std::cout << "Num projection steps : " << project_iter << " "<< dist << " and success : " << success << " " << std::endl;
                    // std::cout << "Num steps : " << project_iter << " and success : " << success << " " << " dist " << dist << " q " << q << " q_new " << q_new << std::endl;
                }

                return success;
            }
    };

}  // namespace vamp::planning
