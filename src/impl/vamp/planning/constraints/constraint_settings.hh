#pragma once

namespace vamp::planning::constraint

{
    struct ConstraintSettings
    {
        int projection_method = 0;
        float descend_rate = 1.0;
        float std_dev_scaling_factor = 0.1F;
        int num_projection_iterations = 15;
        bool insert_all_to_tree = true;

        void display() const
        {
            std::cout << "Projection Method: " << projection_method << ", ";
            std::cout << "Descend Rate: " << descend_rate << ", ";
            std::cout << "Num Projection Iterations: " << num_projection_iterations << ", ";
            std::cout << "Insert All to Tree: " << insert_all_to_tree << "\n";
        }
    };
}  // namespace vamp::planning::constraint