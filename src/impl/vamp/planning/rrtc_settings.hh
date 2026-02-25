#pragma once

namespace vamp::planning
{
    struct RRTCSettings
    {
        float range = 1.;

        bool dynamic_domain = true;
        float radius = 0.5;
        float alpha = 0.0001;
        float min_radius = 1.;

        bool balance = true;
        float tree_ratio = 1.;

        std::size_t max_iterations = 100000;
        std::size_t max_samples = 1000000;
        bool start_tree_first = true;
        int projection_method = 0;

        float descend_rate = 1.0;
        float std_dev_scaling_factor = 0.2F;
        int num_projection_iterations = 25;
        bool insert_all_to_tree = true;


        void display() const
        {
            std::cout << "Range: " << range << ", ";
            std::cout << "Dynamic Domain: " << dynamic_domain << ", ";
            std::cout << "Radius: " << radius << ", ";
            std::cout << "Alpha: " << alpha << ", ";
            std::cout << "Min Radius: " << min_radius << ", ";
            std::cout << "Balance: " << balance << ", ";
            std::cout << "Tree Ratio: " << tree_ratio << ", ";
            std::cout << "Max Iterations: " << max_iterations << ", ";
            std::cout << "Max Samples: " << max_samples << ", ";
            std::cout << "Start Tree First: " << start_tree_first << ", ";
            std::cout << "Projection Method: " << projection_method << ", ";
            std::cout << "Descend Rate: " << descend_rate << ", ";
            std::cout << "Num Projection Iterations: " << num_projection_iterations << ", ";
            std::cout << "Insert All to Tree: " << insert_all_to_tree << "\n";
        }
    };
}  // namespace vamp::planning
