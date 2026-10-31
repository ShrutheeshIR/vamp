#pragma once

#include <vamp/planning/constraints/manifold/constraint.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning::constraint
{
    // Push a configuration out of self-collision using ONE representative sphere per link --
    // the "middle" sphere of that link's collision-sphere list (see cricket's
    // RobotInfo::self_collision_spec) -- for every link pair the robot's SRDF leaves enabled.
    // Each pair contributes one row: the signed distance between the two spheres,
    // |p_a - p_b| - r_a - r_b, hinged at `margin`, so a row is zero (inactive) whenever the
    // spheres are at least `margin` apart and negative (the amount of violation) otherwise --
    // the same hinge-and-mask shape as EnvironmentCollisionConstraint. The sphere positions and
    // their Jacobians come from the generated Robot::self_collision_jacobian; the distance and
    // its gradient are hand-written and chain-ruled through them.
    //
    // The margin band has positive measure, so these rows are never manifold-defining
    // (active_rows() reports false for all of them).
    template <typename Robot, std::size_t rake>
    struct SelfCollisionConstraint final : Constraint<Robot, rake>
    {
        using Block = typename Robot::template ConfigurationBlock<rake>;
        using Row = FloatVector<rake, 1>;

        static constexpr std::size_t dim = Robot::dimension;
        static constexpr std::size_t n_spheres = Robot::n_self_collision_spheres;
        static constexpr std::size_t err_size = Robot::n_self_collision_pairs;
        static constexpr std::size_t jac_size = err_size * dim;

        explicit SelfCollisionConstraint(float margin = 0.01F) : margin_(margin)
        {
        }

        auto squared_error(const Block &q) const noexcept -> Row final
        {
            evaluate(q);

            auto d = solve.err[0] * solve.err[0];
            for (auto i = 1U; i < err_size; ++i)
            {
                d = d + solve.err[i] * solve.err[i];
            }

            return d;
        }

        void step(Block &q, ProjMethod method, float alpha) const noexcept final
        {
            Block gradient;
            switch (method)
            {
                case ProjMethod::BoundedLS:  // no bounded solver for this constraint: InnerLM
                case ProjMethod::InnerLM:
                    Robot::template solve_self_collision_error_lm_inner<rake>(solve, gradient);
                    break;
                case ProjMethod::OuterLM:
                    Robot::template solve_self_collision_error_lm_outer<rake>(solve, gradient);
                    break;
                case ProjMethod::GradDesc:
                    Robot::template solve_self_collision_error_gradient_descent<rake>(solve, gradient);
                    break;
            }

            integrate_step<Robot, rake>(q, gradient, alpha);
        }

        auto n_rows() const noexcept -> std::size_t final
        {
            return err_size;
        }

        void active_rows(bool *rows) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                rows[i] = false;
            }
        }

        void mask_jacobian(const bool *pinned) const noexcept final
        {
            mask_pinned_columns<Robot>(solve, pinned);
        }

        void evaluate_error_jacobian(const Block &q) const noexcept final
        {
            evaluate(q);
        }

        void extract_error_jacobian(std::size_t lane, float *err, float *jac) const noexcept final
        {
            for (auto i = 0U; i < err_size; ++i)
            {
                err[i] = solve.err[{i, lane}];
            }

            for (auto i = 0U; i < jac_size; ++i)
            {
                jac[i] = solve.jac[{i, lane}];
            }
        }

    private:
        void evaluate(const Block &q) const noexcept
        {
            input.q = q;
            Robot::template self_collision_jacobian<rake>(input, positions);

            const auto margin = Row::fill(margin_);
            const auto eps = Row::fill(1e-6F);

            for (auto k = 0U; k < err_size; ++k)
            {
                const auto a = Robot::self_collision_pair_a[k];
                const auto b = Robot::self_collision_pair_b[k];

                const Row dx = positions.pos[3 * a + 0] - positions.pos[3 * b + 0];
                const Row dy = positions.pos[3 * a + 1] - positions.pos[3 * b + 1];
                const Row dz = positions.pos[3 * a + 2] - positions.pos[3 * b + 2];

                // Center coincidence is the only degenerate case: eps keeps the
                // normalization finite (the direction is arbitrary there).
                const auto norm = (dx * dx + dy * dy + dz * dz).sqrt();
                const auto safe_norm = norm.max(eps);
                const auto radii = Row::fill(Robot::self_collision_radii[a] + Robot::self_collision_radii[b]);
                const auto distance = norm - radii;

                const auto nx = dx / safe_norm;
                const auto ny = dy / safe_norm;
                const auto nz = dz / safe_norm;

                // One-sided hinge (only "too close"): zero outside the margin band, the
                // (negative) amount of violation inside it.
                const auto violation = (distance - margin).min(0.F);
                solve.err[k] = violation;

                // Satisfied rows drop out of the Jacobian too (same reasoning as
                // EnvironmentCollisionConstraint): a zero-residual row would otherwise pull
                // the solve toward "hold this pair's distance".
                const auto active = violation != 0.F;

                for (auto j = 0U; j < dim; ++j)
                {
                    const auto dax = positions.jac[(3 * a + 0) * dim + j];
                    const auto day = positions.jac[(3 * a + 1) * dim + j];
                    const auto daz = positions.jac[(3 * a + 2) * dim + j];
                    const auto dbx = positions.jac[(3 * b + 0) * dim + j];
                    const auto dby = positions.jac[(3 * b + 1) * dim + j];
                    const auto dbz = positions.jac[(3 * b + 2) * dim + j];

                    // Chain rule: d(distance)/dq = n . (d p_a/dq - d p_b/dq).
                    solve.jac[k * dim + j] = active & (nx * (dax - dbx) + ny * (day - dby) + nz * (daz - dbz));
                }
            }
        }

        // Input layout of the generated self_collision_jacobian: just the configuration.
        struct Input
        {
            Block q;

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return q[index];
            }
        };

        // Output layout of self_collision_jacobian: d(pos)/dq (3 * n_spheres x dim,
        // row-major), then pos (3 * n_spheres).
        struct Positions
        {
            FloatVector<rake, 3 * n_spheres * dim> jac;
            FloatVector<rake, 3 * n_spheres> pos;

            auto operator[](std::size_t index) noexcept -> Row &
            {
                return (index < 3 * n_spheres * dim) ? jac[index] : pos[index - 3 * n_spheres * dim];
            }

            auto operator[](std::size_t index) const noexcept -> Row
            {
                return (index < 3 * n_spheres * dim) ? jac[index] : pos[index - 3 * n_spheres * dim];
            }
        };

        // Input layout of the generated solvers: d(err)/dq (err_size x dim, row-major), then
        // err (err_size).
        using Solve = SolveBuffer<rake, err_size, jac_size>;

        float margin_;

        mutable Input input;
        mutable Positions positions;
        mutable Solve solve;
    };
}  // namespace vamp::planning::constraint
