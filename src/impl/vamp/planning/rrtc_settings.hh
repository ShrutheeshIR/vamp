#pragma once

namespace vamp::planning
{
    struct RRTCSettings
    {
        float range = 0.1;

        bool dynamic_domain = false;
        float radius = 0.1;
        float alpha = 0.0001;
        float min_radius = 0.01;

        bool balance = true;
        float tree_ratio = 1.;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 100000;
        bool start_tree_first = true;
    };
}  // namespace vamp::planning
