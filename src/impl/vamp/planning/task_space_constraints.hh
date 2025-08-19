#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>
#include <iostream>

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

            double projectStep(const ConfigurationArray &q, ConfigurationArray &q_new, bool update_q = true)
            {
                Eigen::Matrix<float, 6, Eigen::Dynamic> J;
                Eigen::Transform<float, 3, Eigen::Isometry> eef_pose;

                Robot::jacobian_eefk(q, J, eef_pose); // can be parallel from distance
                const Eigen::Vector<float, 6> distance = distanceToConstraintEEF(eef_pose);

                if (update_q)
                {
                    // need to convert jacobian into ref frame
                    const Eigen::Matrix<float, Eigen::Dynamic, 1> update = J.transpose() * (J * J.transpose() + 1e-6 * Eigen::Matrix<float, 6, 6>::Identity()).ldlt().solve(distance);
                    integrateJointConfiguration(q, q_new, update);
                }
                return distance.squaredNorm();

            }

            bool project(const ConfigurationArray &q, ConfigurationArray &q_new)
            {
                double distance = projectStep(q, q_new, false);
                size_t project_iter = 0;
                bool success = false;
                std::cout << distance << " " << std::endl;
                while ((project_iter < max_project_iters) && (distance > tolerance))
                {
                    distance = projectStep(q_new, q_new, true);
                    std::cout << distance << " " << std::endl;
                }
                if (distance < tolerance)
                    success = true;
                
                return success;
                    
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
    template <typename Robot>
    class TaskSpaceConstraint : public RobotConstraint<Robot>
    {
        using Configuration = typename Robot::Configuration;
        using ConfigurationArray = typename Robot::ConfigurationArray;

        protected:

            // 6 is for SE3 space 
            Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference;
            Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world;
            Eigen::Transform<float, 3, Eigen::Isometry> world_frame_w_ref_frame;
            // std::pair<vamp::FloatVector<6>, vamp::FloatVector<6>> bounds;
            std::pair<Eigen::Vector<float, 6>, Eigen::Vector<float, 6>> bounds; 
            bool mComputeErrorFromCenter;


        public:

            TaskSpaceConstraint(
                const Eigen::Transform<float, 3, Eigen::Isometry> eef_pose_w_ref_reference,
                const Eigen::Transform<float, 3, Eigen::Isometry> ref_frame_w_world,
                const std::pair<Eigen::Vector<float, 6>, Eigen::Vector<float, 6>> bounds
            ) : eef_pose_w_ref_reference(eef_pose_w_ref_reference),  ref_frame_w_world(ref_frame_w_world), bounds(bounds)
            {
                world_frame_w_ref_frame = ref_frame_w_world.inverse();
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


                const auto actual_pose = world_frame_w_ref_frame * computed_eef_pose_world_frame;

                //Eigen::Vector3
                const auto translation = actual_pose.translation() - eef_pose_w_ref_reference.translation();
                const auto rot_error = actual_pose.linear() * eef_pose_w_ref_reference.linear().transpose(); // equivalent to putting transpose on actual_pose
                Eigen::AngleAxisf aa(rot_error);

                // Eigen::Vec6
                displacement << aa.axis() * aa.angle(), translation;


                // (TODO) make this 0 if inside the bounds
                for (size_t i = 0; i < 6; i++)
                {
                    if (displacement[i] < bounds.first(i))
                        penalty[i] = displacement[i] - bounds.first(i);
                    else if (displacement[i] > bounds.second(i))
                        penalty[i] = displacement[i] - bounds.second(i);
                    else
                        penalty[i] = 0.0;
                }
                return penalty;
            }

            const Eigen::VectorXf distanceToConstraint(const ConfigurationArray &q)
            {
                const auto fk = Robot::eefk(q);
                return distanceToConstraintEEF(fk);

            }

    };

}