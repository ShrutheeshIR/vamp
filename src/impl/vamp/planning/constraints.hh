#pragma once

#include <memory>

#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    class TaskSpaceConstraint
    {
        protected:
            // 6 is for SE3 space 
            vamp::FloatVector<6> eef_pose;
            vamp::FloatVector<6> min_bound;
            vamp::FloatVector<6> max_bound;

        public:
            void computeJacobian();
            void project();
            void isValid();


    };


    class ConstrainedSampler
}