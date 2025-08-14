#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <Eigen/Geometry>

namespace vamp::planning
{
    template <typename Robot>

    // We will work with the assumption that all the transforms are given to us
    class TaskSpaceConstraint
    {
        protected:

            // 6 is for SE3 space 
            Eigen::Transform<DataT, 3, Eigen::Isometry> eef_pose_w_ref_reference;
            Eigen::Transform<DataT, 3, Eigen::Isometry> ref_frame_w_world;
            Eigen::Transform<DataT, 3, Eigen::Isometry> world_frame_w_ref_frame;
            std::pair<vamp::FloatVector<6>, vamp::FloatVector<6>> bounds;
            bool mComputeErrorFromCenter;


        public:
            void computeJacobian();
            void project();
            void isValid();
            void function();
            void discreteGeodesic();

            void setComputeFromCenter(bool computeFromCenter);
            bool isComputingFromCenter() const;

            void setReferenceFrame(Eigen::Transform<DataT, 3, Eigen::Isometry> referenceFrame);

            void distanceToTSR(Eigen::Transform<DataT, 3, Eigen::Isometry> computed_eef_pose_world_frame)
            {
                const auto actual_pose = world_frame_w_ref_frame * computed_eef_pose_world_frame;
                //Eigen::Vector3
                const auto translation = actual_pose.translation() - eef_pose_w_ref_reference.translation();
                const auto rot_error = actual_pose.linear() * eef_pose_w_ref_reference.linear().transpose(); // equivalent to putting transpose on actual_pose

                Eigen::AngleAxisd aa(rot_error);

                // Eigen::Vec6
                Eigen::Vector6d displacement;
                displacement << aa.axis() * aa.angle(), translation;

            }





    };


    class ConstrainedSampler
}