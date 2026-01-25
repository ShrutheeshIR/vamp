#pragma once

#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <random>
#include <cmath>
#include <iostream>

namespace vamp::optimization
{
    // Computes Gradient of SDF via Central Difference
    template <typename Robot, std::size_t rake>
    inline auto compute_gradient(
        const collision::Environment<FloatVector<rake>> &environment,
        const typename Robot::template ConfigurationBlock<rake> &state,
        float h = 1e-4f) noexcept -> typename Robot::template ConfigurationBlock<rake>
    {
        using ConfigBlock = typename Robot::template ConfigurationBlock<rake>;

        ConfigBlock grad;
        auto h_vec = FloatVector<rake>::fill(h);
        auto inv_2h = FloatVector<rake>::fill(1.0f / (2.0f * h));

        // Create a mutable copy of state to perturb
        ConfigBlock perturbed = state;

        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            auto original_val = perturbed[i];

            // f(x + h)
            perturbed[i] = original_val + h_vec;
            auto f_plus = Robot::sdf(environment, perturbed);

            // f(x - h)
            perturbed[i] = original_val - h_vec;
            auto f_minus = Robot::sdf(environment, perturbed);

            // Restore original
            perturbed[i] = original_val;

            // g = (f_plus - f_minus) / 2h
            grad[i] = (f_plus - f_minus) * inv_2h;
        }
        return grad;
    }

    template <typename Robot, std::size_t rake>
    inline auto project_to_valid(
        typename Robot::ConfigurationBlock<rake> &current_state,
        const collision::Environment<FloatVector<rake>> &environment,
        int steps = 100,
        float learning_rate = 0.5f) noexcept -> typename Robot::template ConfigurationBlock<rake>
    {
        auto lr = vamp::FloatVector<rake>::fill(learning_rate);


        for (int step = 0; step < steps; ++step)
        {
            // Calculate Gradient
            auto grad = compute_gradient<Robot, rake>(environment, current_state);

            // Update Rule: Gradient Ascent
            // q_new = q + lr * grad
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                auto delta = grad[i] * lr;
                current_state[i] = current_state[i] + delta;
            }
        }

        return current_state;
    }
}
