#pragma once
#include <vamp/planning/constraints/constraint_settings.hh>
#include <vamp/planning/rrtc_settings.hh>

namespace vamp::planning
{
    struct CRRTCSettings
    {

        RRTCSettings rrtc_settings;
        vamp::planning::constraint::ConstraintSettings constraint_settings;


        void display() const
        {
            rrtc_settings.display();
            constraint_settings.display();
        }
    };
}  // namespace vamp::planning