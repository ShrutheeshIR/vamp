vamp / src / impl / vamp / planning /
    task_space_constraint_better_code.cc
```
```
// Optimized task space constraint utilities and refactored projection using SIMD helpers
// This file provides reusable, SIMD-friendly building blocks that constraints can use
// to compute errors, jacobians, and perform robust projection steps with better cache
// locality and fewer temporaries.
//
// The design goals:
// - Centralize common logic shared by TaskSpaceConstraint, BimanualTaskSpaceConstraint,
//   SelfCollisionConstraint, FeetTaskSpaceConstraint, and CoMTaskSpaceConstraint.
// - Minimize data movement and branch mispredictions.
// - Use SIMD-aware operations already exposed by FloatVector/VectorInterface.
// - Provide numerically stable line search and damping steps.
//
// Usage: Constraints can delegate their inner projection loops to these utilities,
// passing typed functors/lambdas for "compute_error_and_jacobian" and
// "integrate_configuration" specialized to their models.
//
// Note: This file is careful to avoid directly depending on private fields of the
// constraint classes. It focuses on generic utilities with a small public API.
//
// NOLINTBEGIN(*-magic-numbers)
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <vamp/planning/task_space_constraint.hh>  // for ProjMethod enum

    namespace vamp::planning::constraint
{
    // Small numeric helpers with sane defaults
    struct Numeric
    {
        static constexpr float default_damping = 1e-2f;
        static constexpr float min_step_norm = 1e-6f;
        static constexpr float accept_improve = 0.99f;  // require at least 1% improvement
        static constexpr float backtrack_ratio = 0.5f;  // backtracking multiplier
        static constexpr float min_backtrack = 1e-4f;   // lower bound on backtracking
        static constexpr std::size_t max_linesearch_iters = 16;
    };

    // Compute a per-dimension "inter-lane-distance" ConfigurationBlock using a broadcasted start
    // This avoids element(i) access and remains SIMD-safe across rows.
    template <typename Robot, std::size_t rake>
    inline constexpr auto inter_lane_distance_block(
        const typename Robot::template ConfigurationBlock<rake> &q,
        const typename Robot::Configuration &start) -> typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> out = q;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            // Broadcast the scalar "start[i]" into a 1×rake vector, and compute inter-lane-distance
            const auto start_vec = start.broadcast(i);  // shape-safe
            out[i] = out[i].inter_lane_distance(start_vec);
        }
        return out;
    }

    // Compute a robust norm for a ConfigurationBlock: max(l2_norm per row, or max of lane hmax)
    // This helps detect near-zero steps reliably.
    template <typename Robot, std::size_t rake>
    inline constexpr auto configuration_block_norm(
        const typename Robot::template ConfigurationBlock<rake> &dx) -> float
    {
        float max_norm = 0.0f;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            const float ln = dx[i].l2_norm();
            const float hm = dx[i].hmax();
            const float row_norm = std::max(ln, hm);
            max_norm = std::max(max_norm, row_norm);
        }
        return max_norm;
    }

    // Weighted Jacobian transpose step:
    // Produces a configuration delta "dq" from a Jacobian J and error e, with optional per-dimension weights.
    // J layout is (m × n) where m is stacked task-space rows across EEFs, and n = Robot::dimension.
    template <typename Robot, std::size_t rake, std::size_t m_rows>
    inline constexpr auto jacobian_transpose_step(
        const vamp::FloatVector<rake, m_rows * Robot::dimension> &J,
        const vamp::FloatVector<rake, m_rows> &err,
        const typename Robot::template ConfigurationBlock<rake> &weights  // typically ones
        ) -> typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> dq;
        // For each configuration dimension j, accumulate sum_i (J_ij * err_i)
        // Memory layout: J is grouped per dimension j, each having m_rows entries.
        for (std::size_t j = 0; j < Robot::dimension; ++j)
        {
            vamp::FloatVector<rake, 1> acc(0.0f);
            for (std::size_t i = 0; i < m_rows; ++i)
            {
                // J[i + j * m_rows] is the coefficient for (row i, column j)
                acc = acc + (J[i + j * m_rows] * err[i]);
            }
            dq[j] = acc * weights[j];
        }
        return dq;
    }

    // Levenberg-Marquardt style damping:
    // Scales the step according to a damping factor, while preserving direction.
    template <typename Robot, std::size_t rake>
    inline constexpr auto damp_step(
        const typename Robot::template ConfigurationBlock<rake> &dq, const float lambda) ->
        typename Robot::template ConfigurationBlock<rake>
    {
        typename Robot::template ConfigurationBlock<rake> out;
        const float scale = 1.0f / (1.0f + lambda);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            out[i] = dq[i] * scale;
        }
        return out;
    }

    // Backtracking line-search:
    // Try scaling the step by successive powers of "ratio" to find improvement.
    // Caller supplies "evaluate" which maps a candidate configuration to a distance scalar.
    template <typename Robot, std::size_t rake, typename EvaluateFn>
    inline auto backtracking_linesearch(
        const typename Robot::template ConfigurationBlock<rake> &q,
        const typename Robot::template ConfigurationBlock<rake> &dir,
        const EvaluateFn &evaluate,
        const float current_dist,
        const float ratio = Numeric::backtrack_ratio,
        const float min_scale =
            Numeric::min_backtrack) -> std::pair<typename Robot::template ConfigurationBlock<rake>, float>
    {
        typename Robot::template ConfigurationBlock<rake> best_q = q;
        float best_dist = current_dist;

        float scale = 1.0f;
        for (std::size_t k = 0; k < Numeric::max_linesearch_iters; ++k)
        {
            // q_candidate = q + scale * dir
            typename Robot::template ConfigurationBlock<rake> q_candidate;
            for (std::size_t i = 0; i < Robot::dimension; ++i)
            {
                q_candidate[i] = q[i] + (dir[i] * scale);
            }

            const float cand_dist = evaluate(q_candidate);
            if (cand_dist <= Numeric::accept_improve * best_dist)
            {
                best_q = q_candidate;
                best_dist = cand_dist;
                break;
            }

            scale *= ratio;
            if (scale < min_scale)
            {
                break;
            }
        }

        return {best_q, best_dist};
    }

    // Generalized projector orchestrator:
    // - compute_error_and_jacobian(q, out_err, out_J) should fill err (m_rows) and J (m_rows × dimension)
    // - integrate_configuration(q) should update any internal kinematic caches for the Robot
    // - evaluate_distance(q) returns a scalar constraint violation
    // - The method adapts to InnerLM, OuterLM, GradDesc via "method" argument.
    template <
        typename Robot,
        std::size_t rake,
        std::size_t m_rows,
        typename ComputeErrorJacobianFn,
        typename IntegrateConfigurationFn,
        typename EvaluateDistanceFn>
    inline auto project_configuration(
        typename Robot::template ConfigurationBlock<rake> q,
        const typename Robot::template ConfigurationBlock<rake> &q_start,
        const ComputeErrorJacobianFn &compute_error_and_jacobian,
        const IntegrateConfigurationFn &integrate_configuration,
        const EvaluateDistanceFn &evaluate_distance,
        const ProjMethod method,
        const std::size_t max_iters = 64,
        const float damping =
            Numeric::default_damping) -> std::pair<typename Robot::template ConfigurationBlock<rake>, bool>
    {
        // Precompute a constant weight vector of ones
        typename Robot::template ConfigurationBlock<rake> ones;
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            ones[i] = vamp::FloatVector<rake, 1>(1.0f);
        }

        bool success = false;
        float prev_dist = evaluate_distance(q);

        for (std::size_t iter = 0; iter < max_iters; ++iter)
        {
            // Update kinematics/caches
            integrate_configuration(q);

            // Compute error (m_rows) and jacobian (m_rows × dimension) at current q
            vamp::FloatVector<rake, m_rows> err;
            vamp::FloatVector<rake, m_rows * Robot::dimension> J;
            compute_error_and_jacobian(q, err, J);

            // Direction via Jacobian transpose step
            auto dir = jacobian_transpose_step<Robot, rake, m_rows>(J, err, ones);

            // If using LM, damp the direction
            if (method == ProjMethod::InnerLM || method == ProjMethod::OuterLM)
            {
                dir = damp_step<Robot, rake>(dir, damping);
            }

            // If the step is too small, we consider convergence
            const float dir_norm = configuration_block_norm<Robot, rake>(dir);
            if (dir_norm < Numeric::min_step_norm)
            {
                success = true;
                break;
            }

            // Evaluate current distance
            const float current_dist = prev_dist;

            // Gradient descent: take a step and then backtrack for improvement
            auto [q_candidate, new_dist] =
                backtracking_linesearch<Robot, rake>(q, dir, evaluate_distance, current_dist);

            // Accept if improved
            if (new_dist < current_dist)
            {
                q = q_candidate;
                prev_dist = new_dist;
            }
            else
            {
                // No improvement – stop if we're already close
                if (current_dist < Numeric::min_step_norm)
                {
                    success = true;
                }
                break;
            }

            // Optional: early stop if sufficiently close to start or previous
            const auto q_dist_from_prev = configuration_block_norm<Robot, rake>(dir);
            const auto q_dist_from_start = configuration_block_norm<Robot, rake>(
                inter_lane_distance_block<Robot, rake>(q, typename Robot::Configuration(q_start)));

            if (q_dist_from_prev < Numeric::min_step_norm && q_dist_from_start < Numeric::min_step_norm)
            {
                success = true;
                break;
            }
        }

        return {q, success};
    }

    // Convenience helpers that wrap the orchestrator for each method type,
    // letting existing constraints opt-in gradually without changing their internal logic.
    template <
        typename Robot,
        std::size_t rake,
        std::size_t m_rows,
        typename ComputeErrorJacobianFn,
        typename IntegrateConfigurationFn,
        typename EvaluateDistanceFn>
    inline auto project_inner_lm(
        const typename Robot::template ConfigurationBlock<rake> &q_init,
        const typename Robot::template ConfigurationBlock<rake> &q_start,
        const ComputeErrorJacobianFn &compute_error_and_jacobian,
        const IntegrateConfigurationFn &integrate_configuration,
        const EvaluateDistanceFn &evaluate_distance,
        const std::size_t max_iters = 64,
        const float damping =
            Numeric::default_damping) -> std::pair<typename Robot::template ConfigurationBlock<rake>, bool>
    {
        return project_configuration<Robot, rake, m_rows>(
            q_init,
            q_start,
            compute_error_and_jacobian,
            integrate_configuration,
            evaluate_distance,
            ProjMethod::InnerLM,
            max_iters,
            damping);
    }

    template <
        typename Robot,
        std::size_t rake,
        std::size_t m_rows,
        typename ComputeErrorJacobianFn,
        typename IntegrateConfigurationFn,
        typename EvaluateDistanceFn>
    inline auto project_outer_lm(
        const typename Robot::template ConfigurationBlock<rake> &q_init,
        const typename Robot::template ConfigurationBlock<rake> &q_start,
        const ComputeErrorJacobianFn &compute_error_and_jacobian,
        const IntegrateConfigurationFn &integrate_configuration,
        const EvaluateDistanceFn &evaluate_distance,
        const std::size_t max_iters = 64,
        const float damping =
            Numeric::default_damping) -> std::pair<typename Robot::template ConfigurationBlock<rake>, bool>
    {
        return project_configuration<Robot, rake, m_rows>(
            q_init,
            q_start,
            compute_error_and_jacobian,
            integrate_configuration,
            evaluate_distance,
            ProjMethod::OuterLM,
            max_iters,
            damping);
    }

    template <
        typename Robot,
        std::size_t rake,
        std::size_t m_rows,
        typename ComputeErrorJacobianFn,
        typename IntegrateConfigurationFn,
        typename EvaluateDistanceFn>
    inline auto project_grad_desc(
        const typename Robot::template ConfigurationBlock<rake> &q_init,
        const typename Robot::template ConfigurationBlock<rake> &q_start,
        const ComputeErrorJacobianFn &compute_error_and_jacobian,
        const IntegrateConfigurationFn &integrate_configuration,
        const EvaluateDistanceFn &evaluate_distance,
        const std::size_t max_iters =
            64) -> std::pair<typename Robot::template ConfigurationBlock<rake>, bool>
    {
        return project_configuration<Robot, rake, m_rows>(
            q_init,
            q_start,
            compute_error_and_jacobian,
            integrate_configuration,
            evaluate_distance,
            ProjMethod::GradDesc,
            max_iters,
            /*damping*/ 0.0f);
    }
}  // namespace vamp::planning::constraint
// NOLINTEND(*-magic-numbers)