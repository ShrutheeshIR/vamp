#pragma once

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/vector/math.hh>
#include <iostream>
#include <iomanip>

namespace vamp::planning
{

    template <typename Robot, std::size_t rake>
    class TaskSpaceConstraint
    {
        using ConfigurationBlock = typename Robot::ConfigurationBlock<rake>;

    public:
        using Vec = vamp::FloatVector<rake, 1>;

        // static constexpr std::size_t rTe_size = 7 * Robot::n_eef;
        // static constexpr std::size_t wTr_size = 7 * Robot::n_eef;
        // static constexpr std::size_t lb_size = 6 * Robot::n_eef;
        // static constexpr std::size_t rb_size = 6 * Robot::n_eef;

        static constexpr std::size_t error_size = 6 * Robot::n_eef;
        static constexpr std::size_t J_error_size = error_size * Robot::dimension;

        struct JacErrorView
        {
            std::span<Vec, J_error_size> j;
            std::span<Vec, error_size> e;
        };

        struct ConstJacErrorView
        {
            std::span<const Vec, J_error_size> j;
            std::span<const Vec, error_size> e;
        };

        TaskSpaceConstraint(
            std::array<std::array<float, 7>, Robot::n_eef> &eef_pose_w_ref_reference,  // qw, qx, qy, qz, tx,
                                                                                       // ty, tz
            std::array<std::array<float, 7>, Robot::n_eef> &ref_frame_w_world,  // qw, qx, qy, qz, tx, ty, tz
            const std::array<float, 6 * Robot::n_eef> &lower_bound,
            const std::array<float, 6 * Robot::n_eef> &upper_bound)
        {
            std::size_t offset = Robot::dimension;
            for (std::size_t ee = 0; ee < numeef; ++ee)
            {
                for (std::size_t j = 0; j < 7; ++j)
                {
                    tsr_inp[offset++] = Vec(eef_pose_w_ref_reference[ee][j]);
                }

                for (std::size_t j = 0; j < 7; ++j)
                {
                    tsr_inp[offset++] = Vec(ref_frame_w_world[ee][j]);
                }

                for (std::size_t j = 0; j < 6; ++j)
                {
                    tsr_inp[offset++] = Vec(lower_bound[ee][j]);
                }

                for (std::size_t j = 0; j < 6; ++j)
                {
                    tsr_inp[offset++] = Vec(upper_bound[ee][j]);
                }
            }
        }

        inline void update_q(const ConfigurationBlock &q_block)
        {
            // Copy each scalar from q_block into the first q_size entries of tsr_inp
            for (std::size_t i = 0; i < q_size; ++i)
            {
                tsr_inp[i] = q_block[i];  // assumes operator[] exists on FloatVector
            }
        }

        // I need 3 main methods
        // distanceToConstraint(q) - calls Robot::error_function(tsr_inp) returns J and e as a single array
        // compute_gradient(q) - calls distanceToConstraint(q) and then calls
        // Robot::compute_gradient(j_e_array) projectStep(q) - calls grad = compute_gradient(q) and then calls
        // integrate_gradient(q, grad, alpha)
        //
        inline auto distanceToConstraint(const ConfigurationBlock &q_block)
        {
            update_q(q_block);
            Robot::template tsr_error<rake>(tsr_inp, j_e_array);
            Vec distance{};  // all lanes zero

            for (std::size_t i = 0; i < error_size; ++i)
            {
                // Compute element-wise difference: e[i] - d[i]
                Vec diff = array_out.e[i] - array_in[q_size + R_size + i];

                // Compute which eef this element belongs to
                std::size_t ee = i / 6;   // 6 error rows per eef
                std::size_t idx = i % 6;  // index within lb/ub

                // Compute offsets in array_in
                std::size_t base = q_size + ee * (7 + 7 + 6 + 6);
                Vec lbi = array_in[base + 14 + idx];      // lb offset
                Vec ubi = array_in[base + 14 + 6 + idx];  // ub offset

                j_e_array.e[i] =
                    (j_e_array.e[i] - tsr_inp[lbi]).min(0.F) + (j_e_array.e[i] - tsr_inp[ubi]).max(0.F);

                // Square element-wise
                Vec squared = clamped * clamped;

                // Accumulate along array dimension (element-wise sum of Vec)
                distance = distance + squared;
            }

            return distance;
        }

        inline auto print_tsr_error(const ConfigurationBlock &q_block)
        {
            std::stringstream ss;
            ss << "TSR Error for block " << std::endl;
            for (std::size_t i = 0; i < error_size; ++i)
            {
                ss << "Error " << i << ": " << j_e_array.e[i] << std::endl;
            }
            ss << "TSR Jacobian for block " << std::endl;
            for (std::size_t i = 0; i < J_error_size; ++i)
            {
                ss << "Jacobian " << i << ": " << j_e_array.j[i] << std::endl;
            }
            return ss.str();
        }

        inline auto compute_gradient(const ConfigurationBlock &q_block)
        {
            std::vector<Vec> gradient(J_error_size);
            for (std::size_t i = 0; i < J_error_size; ++i)
            {
                gradient[i] = j_e_array.j[i] * j_e_array.e[i];
            }
            return gradient;
        }

        std::span<Vec, error_size> e()
        {
            return std::span<Vec, error_size>(j_e_array.end() - error_size, error_size);
        }

        std::span<const Vec, error_size> e() const
        {
            return std::span<const Vec, error_size>(j_e_array.end() - error_size, error_size);
        }

        // -----------------------------
        // Access full output (J + e)
        // -----------------------------
        std::span<Vec, J_error_size + error_size> full_output()
        {
            return j_e_array;
        }

    private:
        Vector3 position_;
        Vector3 orientation_;
    };
}  // namespace vamp::planning
