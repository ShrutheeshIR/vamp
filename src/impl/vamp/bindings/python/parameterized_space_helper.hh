#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <vamp/bindings/python/array_helpers.hh>
#include <vamp/bindings/python/space_helper.hh>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/parameterized_local_planner.hh>
#include <vamp/planning/planner.hh>
#include <vamp/planning/planners/aorrtc.hh>
#include <vamp/planning/planners/aorrtc_settings.hh>
#include <vamp/planning/planners/grrtstar.hh>
#include <vamp/planning/planners/grrtstar_settings.hh>
#include <vamp/planning/planners/rrtc.hh>
#include <vamp/planning/planners/rrtc_settings.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/simplify.hh>
#include <vamp/planning/simplify_settings.hh>
#include <vamp/random/halton.hh>
#include <vamp/random/rng.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

#if defined(__x86_64__)
#include <vamp/random/xorshift.hh>
#endif

#include <nanobind/nanobind.h>

// Not every ParameterizedSpace needs a "mid pose" concept: it exists for bimanual/
// mobile-base spaces (e.g. RBY1, whose t_mid_left/t_mid_right hand offsets are derived
// from a reference whole-body configuration) but not single-arm ones (e.g. IiwaMarker,
// whose eef pose IS the task-space pose already). Detected via SFINAE so
// ParameterizedSpaceTraits::compute_mid_pose and its binding can each stay a no-op for
// robots without it, rather than hard-erroring at compile time.
VAMP_DEFINE_HAS_METHOD(compute_mid_pose)

// LeaderFollowerSpace's counterpart of compute_mid_pose: derives a single fixed
// leader-to-follower hand offset (rel_pose) from a reference whole-body configuration,
// instead of a shared mid-frame offset. See BimanualIiwa::LeaderFollowerSpace::rel_pose.
VAMP_DEFINE_HAS_METHOD(compute_rel_pose)

// GCP (self-motion-manifold branch selector) is RBY1-specific: each rainbow arm has its
// own (elbow_sel, shoulder_sel, wrist_sel) triple, set independently per arm via
// Space::set_gcp(left, right). BimanualIiwa::ParameterizedSpace has the same two-triple
// shape under a different name (set_gc(left, right)) -- see has_set_gc_v below.
VAMP_DEFINE_HAS_METHOD(set_gcp)

// Single-triple self-motion-manifold branch selector, set via Space::set_smm(gc) --
// IiwaMarker's and BimanualIiwa::LeaderFollowerSpace's shape (one arm's IK, one triple),
// distinct from the two-triple set_gcp/set_gc bimanual shape above.
VAMP_DEFINE_HAS_METHOD(set_smm)

// BimanualIiwa::ParameterizedSpace's two-triple counterpart of set_gcp; see set_gcp's
// comment above for why this isn't unified with it (different method name, same shape).
VAMP_DEFINE_HAS_METHOD(set_gc)

// Detects LeaderFollowerSpace's rel_pose data member directly (decltype(T::rel_pose) is
// well-formed for a static data member exactly like it is for a static method), so a
// literal offset can be assigned in place -- for problems (e.g. a rigid dual-arm carry)
// that specify the fixed leader-to-follower offset directly instead of deriving it from
// a reference whole-body pose via compute_rel_pose.
VAMP_DEFINE_HAS_METHOD(rel_pose)

namespace vamp::binding
{
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;

    namespace detail
    {
        // Detects whether Space exposes a templated eefs_collision_free<rake>(environment,
        // block) utility. Present on ParameterizedSpace/RBY1/IiwaMarker, but not required --
        // it's an IK-free prefilter never called by ParameterizedLocalPlanner itself (see
        // parameterized_local_planner.hh's resolve_and_check_impl, where the equivalent call
        // is commented out), only by the eefs_collision_free() python binding below. Spaces
        // without one (e.g. BimanualIiwa::LeaderFollowerSpace) simply don't expose that
        // prefilter to python; planning is unaffected.
        template <typename Space, std::size_t r, typename = void>
        struct has_eefs_collision_free : std::false_type
        {
        };

        template <typename Space, std::size_t r>
        struct has_eefs_collision_free<
            Space,
            r,
            std::void_t<decltype(Space::template eefs_collision_free<r>(
                std::declval<const vamp::collision::Environment<vamp::FloatVector<r>> &>(),
                std::declval<const typename Space::template StateBlock<r> &>()))>> : std::true_type
        {
        };

        // Detects whether Space exposes the per-lane resolve_block_mask<rake>(block) (a validity
        // mask per lane instead of resolve_block's single "every lane valid" bool), which the
        // batched resolve_sweep below needs.
        template <typename Space, typename = void>
        struct has_resolve_block_mask : std::false_type
        {
        };

        template <typename Space>
        struct has_resolve_block_mask<
            Space,
            std::void_t<decltype(Space::template resolve_block_mask<rake>(
                std::declval<const typename Space::template StateBlock<rake> &>()))>> : std::true_type
        {
        };
    }  // namespace detail

    // Traits for planning over Robot::ParameterizedSpace: a task-space parameterization
    // that IK-resolves into Robot's own ambient Configuration space. Mirrors the shape of
    // StaticRobotTraits (robot_helper.hh) closely enough to reuse bind_path_class,
    // bind_path_io, bind_planning_result, and bind_sampler from api_binder.hh unmodified,
    // but only supports the local-planner-aware planners (rrtc/aorrtc/grrtstar) -- PRM/FCIT
    // don't take a local planner and so can't resolve task-space states into the ambient
    // robot.
    template <
        typename Robot,
        typename SpaceT = typename Robot::ParameterizedSpace,
        typename Input = SpaceNDArrayInput<Robot, SpaceT>>
    struct ParameterizedSpaceTraits
    {
        using RobotType = Robot;
        using Space = SpaceT;
        using Ambient = typename Space::Ambient;

        using Cfg = typename Input::Type;
        using Pth = std::vector<Cfg>;
        using Path = vamp::planning::Path<Robot, Space>;
        using PlanningResult = vamp::planning::PlanningResult<Robot, Space>;
        using Sampler = vamp::rng::RNG<Robot, Space>;
        using Env = vamp::collision::Environment<float>;
        using EnvVec = vamp::collision::Environment<vamp::FloatVector<rake>>;

        using Configuration = typename Space::State;
        using RNG = vamp::rng::RNG<Robot, Space>;
        using LocalPlanner =
            vamp::planning::constraint::ParameterizedLocalPlanner<Robot, rake, Robot::resolution, Space>;

        // Explicit specialization of a nested member template isn't legal while the
        // enclosing ParameterizedSpaceTraits is still a template, so pick the planner type
        // via std::conditional_t instead of a specialized helper struct.
        template <vamp::planning::Planner P>
        using TaskPlannerT = std::conditional_t<
            P == vamp::planning::Planner::RRTC,
            vamp::planning::RRTC<Robot, rake, Robot::resolution, Space>,
            std::conditional_t<
                P == vamp::planning::Planner::AORRTC,
                vamp::planning::AORRTC<Robot, rake, Robot::resolution, Space>,
                vamp::planning::GRRTStar<Robot, rake, Robot::resolution, Space>>>;

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_single(
            const Cfg &start,
            const Cfg &goal,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            return TaskPlannerT<P>::template solve<LocalPlanner>(
                Input::to(start), Input::to(goal), EnvVec(env), s, rng, LocalPlanner());
        }

        template <vamp::planning::Planner P, typename Settings>
        static auto solve_multi(
            const Cfg &start,
            const Pth &goals,
            const Env &env,
            const Settings &s,
            std::shared_ptr<Sampler> rng) -> PlanningResult
        {
            std::vector<Configuration> goals_v;
            goals_v.reserve(goals.size());
            for (const auto &g : goals)
            {
                goals_v.emplace_back(Input::to(g));
            }

            return TaskPlannerT<P>::template solve<LocalPlanner>(
                Input::to(start), goals_v, EnvVec(env), s, rng, LocalPlanner());
        }

        // Distinct from the ambient robot's simplify() (which takes SimplifySettings and runs
        // the full bspline/reduce/perturb pipeline): this wraps the raw shortcut_path the
        // task-space demo uses, taking ShortcutSettings. Mutates path in place.
        static auto shortcut(
            Path &path,
            const Env &env,
            const vamp::planning::ShortcutSettings &settings) -> bool
        {
            return vamp::planning::shortcut_path<Robot, rake, Robot::resolution, LocalPlanner, Space>(
                path, EnvVec(env), settings, LocalPlanner());
        }

        // IK-resolve a single task-space state into the ambient robot's Configuration.
        // Returns (valid, ambient_configuration); does not collision-check the result --
        // callers that need that should follow up with the ambient robot's validate().
        //
        // Returns a plain std::array<float, Ambient::dimension> rather than
        // Ambient::ConfigurationArray: nanobind's std::array caster (nanobind/stl/array.h)
        // is specialized on std::array<Type, Size> exactly, and ConfigurationArray -- a
        // struct that derives from std::array<FloatT, dimension> to carry alignas -- is a
        // distinct type nanobind has no caster for, so returning it directly makes this
        // uncallable from python ("Unable to convert function return value to a Python
        // type!") despite compiling and generating a signature. Both are the same bytes,
        // so this is a lossless conversion.
        static auto resolve(const Cfg &state) -> std::pair<bool, std::array<float, Ambient::dimension>>
        {
            const auto s = Input::to(state);
            typename Space::template StateBlock<rake> block;
            for (std::size_t i = 0; i < Space::dimension; ++i)
            {
                block[i] = s.broadcast(i);
            }

            auto [valid, ambient_block] = Space::template resolve_block<rake>(block);

            std::array<float, Ambient::dimension> ambient;
            for (std::size_t i = 0; i < Ambient::dimension; ++i)
            {
                ambient[i] = ambient_block[{i, 0}];
            }

            return {valid, ambient};
        }

        // Batched resolve of ONE task-space pose over many values of the free parameter psi on
        // a single IK branch `gc` ((elbow_sel, shoulder_sel, wrist_sel), see set_smm): the
        // self-motion sweep. The psi values fill the SIMD lanes `rake` at a time (the last
        // block is padded by repeating its last psi), so a sweep of n values costs n / rake
        // IK evaluations instead of n broadcast calls. Returns (valid (n,), q (n, dim)) with
        // valid = 1.0 where the IK has a solution on this branch and the result is inside the
        // joint limits (resolve_block_mask's conditions) and, with `check_collision`, the
        // resolved configuration is collision-free against `env` and the robot itself
        // (Ambient::fkcc). q rows of invalid entries are meaningless. The collision check
        // tests a whole block at once (fkcc collapses its lanes into one bool) and only falls
        // back to one call per valid lane when that block reports a collision.
        static auto resolve_sweep(
            const std::array<float, 7> &pose,
            const nb::ndarray<const float, nb::ndim<1>, nb::c_contig, nb::device::cpu> &psi,
            const std::array<float, 3> &gc,
            const Env &env,
            bool check_collision)
            -> std::pair<nb::ndarray<nb::numpy, float, nb::device::cpu>, nb::ndarray<nb::numpy, float, nb::device::cpu>>
        {
            if constexpr (detail::has_resolve_block_mask<Space>::value and has_set_smm_v<Space>)
            {
                using V = vamp::FloatVector<rake, 1>;
                constexpr std::size_t dim = Ambient::dimension;

                const std::size_t n = psi.shape(0);
                const float *psi_data = psi.data();
                std::vector<float> q_out(n * dim, 0.F);
                std::vector<float> ok_out(n, 0.F);

                Space::set_smm(gc);
                const EnvVec env_vec(env);

                for (std::size_t start = 0; start < n; start += rake)
                {
                    const std::size_t count = std::min<std::size_t>(rake, n - start);

                    std::array<float, rake> lane_psi{};
                    for (std::size_t l = 0; l < rake; ++l)
                    {
                        lane_psi[l] = psi_data[start + std::min(l, count - 1)];
                    }

                    typename Space::template StateBlock<rake> x;
                    for (std::size_t k = 0; k < 7; ++k)
                    {
                        x[k] = V::fill(pose[k]);
                    }

                    x[7] = V(lane_psi);

                    auto [valid, q] = Space::template resolve_block_mask<rake>(x);

                    // A true lane of the comparison mask is all ones: and-ing with 1.0 turns
                    // it into 1.0 (false lanes into 0.0).
                    const V ok = valid & V::fill(1.F);
                    std::array<bool, rake> lane_ok{};
                    bool any_valid = false;
                    for (std::size_t l = 0; l < count; ++l)
                    {
                        lane_ok[l] = ok[{0, l}] > 0.5F;
                        any_valid = any_valid or lane_ok[l];
                    }

                    if (check_collision and any_valid and
                        not Ambient::template fkcc<rake>(env_vec, q))
                    {
                        for (std::size_t l = 0; l < count; ++l)
                        {
                            if (not lane_ok[l])
                            {
                                continue;
                            }

                            typename Ambient::template ConfigurationBlock<rake> single;
                            for (std::size_t i = 0; i < dim; ++i)
                            {
                                single[i] = V::fill(q[{i, l}]);
                            }

                            lane_ok[l] = Ambient::template fkcc<rake>(env_vec, single);
                        }
                    }

                    for (std::size_t l = 0; l < count; ++l)
                    {
                        ok_out[start + l] = lane_ok[l] ? 1.F : 0.F;
                        for (std::size_t i = 0; i < dim; ++i)
                        {
                            q_out[((start + l) * dim) + i] = q[{i, l}];
                        }
                    }
                }

                return {make_ndarray<1>(ok_out.data(), {n}), make_ndarray<2>(q_out.data(), {n, dim})};
            }
            else
            {
                (void) pose;
                (void) psi;
                (void) gc;
                (void) env;
                (void) check_collision;
                throw std::runtime_error(
                    "this task space has no batched resolve (resolve_block_mask + set_smm)");
            }
        }

        // IK-free prefilter: hand geometry (+ any attachments) vs. environment/each other.
        // Not a substitute for a full collision check on the resolved ambient configuration.
        // Only present on spaces with an eefs_collision_free concept; see
        // detail::has_eefs_collision_free above.
        static auto eefs_collision_free(const Cfg &state, const Env &env) -> bool
        {
            if constexpr (detail::has_eefs_collision_free<Space, rake>::value)
            {
                const auto s = Input::to(state);
                typename Space::template StateBlock<rake> block;
                for (std::size_t i = 0; i < Space::dimension; ++i)
                {
                    block[i] = s.broadcast(i);
                }

                return Space::template eefs_collision_free<rake>(EnvVec(env), block);
            }
            else
            {
                (void) state;
                (void) env;
                throw std::runtime_error("this task space has no eefs_collision_free concept");
            }
        }

        // Derive t_mid_left/t_mid_right (thread-local to Space) from a reference whole-body
        // ambient configuration; the normal way to set them before planning. Only present
        // on spaces with a mid-pose concept (e.g. RBY1); see has_compute_mid_pose_v above.
        // The binder (parameterized_space_binder.hh) only exposes this to python when the
        // if constexpr branch below is taken, so the else branch is unreachable in
        // practice -- it exists only so this method stays well-formed to reference (e.g.
        // take the address of) for spaces without compute_mid_pose.
        //
        // Takes a plain std::array<float, Ambient::dimension> rather than
        // Ambient::ConfigurationArray for the same reason resolve() returns one instead
        // of the latter -- see the comment there.
        static void compute_mid_pose(const std::array<float, Ambient::dimension> &ambient)
        {
            if constexpr (has_compute_mid_pose_v<Space>)
            {
                typename Ambient::ConfigurationArray q;
                for (std::size_t i = 0; i < Ambient::dimension; ++i)
                {
                    q[i] = ambient[i];
                }
                Space::compute_mid_pose(q);
            }
            else
            {
                (void) ambient;
                throw std::runtime_error("this task space has no mid-pose concept");
            }
        }

        // Derive rel_pose (thread-local to Space) from a reference whole-body ambient
        // configuration; LeaderFollowerSpace's counterpart of compute_mid_pose. Only
        // present on spaces with a compute_rel_pose concept; see has_compute_rel_pose_v
        // above. Same nanobind-caster rationale as compute_mid_pose for the plain
        // std::array<float, Ambient::dimension> parameter.
        static void compute_rel_pose(const std::array<float, Ambient::dimension> &ambient)
        {
            if constexpr (has_compute_rel_pose_v<Space>)
            {
                typename Ambient::ConfigurationArray q;
                for (std::size_t i = 0; i < Ambient::dimension; ++i)
                {
                    q[i] = ambient[i];
                }
                Space::compute_rel_pose(q);
            }
            else
            {
                (void) ambient;
                throw std::runtime_error("this task space has no rel-pose concept");
            }
        }

        // Directly overwrite rel_pose (thread-local to Space) with a caller-supplied fixed
        // (x, y, z, qx, qy, qz, qw) leader-to-follower offset, bypassing compute_rel_pose's
        // FK derivation. Only present on spaces with a rel_pose member; see has_rel_pose_v
        // above.
        static void set_rel_pose(const std::array<float, 7> &pose)
        {
            if constexpr (has_rel_pose_v<Space>)
            {
                Space::rel_pose = pose;
            }
            else
            {
                (void) pose;
                throw std::runtime_error("this task space has no rel_pose concept");
            }
        }

        // Overwrite the static-stability support polygon LocalPlanner checks the resolved
        // ambient center of mass against (see ParameterizedLocalPlanner::set_support_polygon).
        // Only meaningful on spaces with a compute_com utility (e.g. RBY1); on spaces without
        // one, com_within_support_polygon is a no-op (always true) regardless of what the
        // polygon is set to, so the binder (parameterized_space_binder.hh) doesn't expose this
        // at all rather than expose a setter with no effect. Same has_compute_com<Space, rake>
        // gate ParameterizedLocalPlanner::com_within_support_polygon itself uses.
        static void set_support_polygon(const std::vector<std::array<float, 2>> &polygon_xy)
        {
            if constexpr (vamp::planning::constraint::detail::has_compute_com<Space, rake>::value)
            {
                LocalPlanner::set_support_polygon(polygon_xy);
            }
            else
            {
                (void) polygon_xy;
                throw std::runtime_error("this task space has no center-of-mass/support-polygon concept");
            }
        }

        // Set the GCP (elbow/shoulder/wrist self-motion-manifold branch) selector shared by
        // every lane -- the "many task-space states, one shared branch" mode
        // ParameterizedLocalPlanner/resolve_and_check use while extending the tree, where
        // the whole planning problem is meant to stay on a single branch. `left`/`right`
        // are each (elbow_sel, shoulder_sel, wrist_sel), matching Space::set_gcp. Only
        // present on spaces with a GCP concept (currently RBY1); see has_set_gcp_v above.
        static void set_gcp(const std::array<float, 3> &left, const std::array<float, 3> &right)
        {
            if constexpr (has_set_gcp_v<Space>)
            {
                Space::set_gcp(left, right);
            }
            else
            {
                (void) left;
                (void) right;
                throw std::runtime_error("this task space has no GCP (self-motion-manifold branch) concept");
            }
        }

        // Two-triple counterpart of set_gcp, for spaces that name it set_gc instead (e.g.
        // BimanualIiwa::ParameterizedSpace). Only present on spaces with a set_gc concept;
        // see has_set_gc_v above.
        static void set_gc(const std::array<float, 3> &left, const std::array<float, 3> &right)
        {
            if constexpr (has_set_gc_v<Space>)
            {
                Space::set_gc(left, right);
            }
            else
            {
                (void) left;
                (void) right;
                throw std::runtime_error("this task space has no set_gc (self-motion-manifold branch) concept");
            }
        }

        // Set the single-triple self-motion-manifold branch selector shared by every lane
        // (e.g. IiwaMarker's and BimanualIiwa::LeaderFollowerSpace's smm). Only present on
        // spaces with a set_smm concept; see has_set_smm_v above.
        static void set_smm(const std::array<float, 3> &gc)
        {
            if constexpr (has_set_smm_v<Space>)
            {
                Space::set_smm(gc);
            }
            else
            {
                (void) gc;
                throw std::runtime_error("this task space has no set_smm (self-motion-manifold branch) concept");
            }
        }

        static auto make_halton() -> std::shared_ptr<Sampler>
        {
            return std::make_shared<vamp::rng::Halton<Robot, Space>>();
        }

        static auto make_xorshift(std::uint64_t seed) -> std::shared_ptr<Sampler>
        {
#if defined(__x86_64__)
            return (seed == 0) ? std::make_shared<vamp::rng::XORShift<Robot, Space>>() :
                                 std::make_shared<vamp::rng::XORShift<Robot, Space>>(seed, seed + 1);
#else
            throw std::runtime_error("XORShift is not supported on non-x86 systems!");
#endif
        }

        // Path element access (path_get/set/append/insert/numpy), for bind_path_class /
        // bind_path_io.
        static auto path_get(const Path &p, std::size_t i) -> Cfg
        {
            return Input::from(p[i]);
        }

        static void path_set(Path &p, std::size_t i, const Cfg &c)
        {
            p[i] = Input::to(c);
        }

        static void path_append(Path &p, const Cfg &c)
        {
            p.emplace_back(Input::to(c));
        }

        static void path_insert(Path &p, std::size_t i, const Cfg &c)
        {
            p.insert(p.begin() + static_cast<std::ptrdiff_t>(i), Input::to(c));
        }

        static auto path_numpy(const Path &p)
        {
            return make_ndarray_filled<2>(
                {p.size(), Space::dimension},
                [&](float *buf)
                {
                    for (std::size_t i = 0; i < p.size(); ++i)
                    {
                        auto arr = p[i].to_array();
                        for (std::size_t j = 0; j < Space::dimension; ++j)
                        {
                            buf[(i * Space::dimension) + j] = arr[j];
                        }
                    }
                });
        }

        static auto path_validate(Path &, const Env &) -> bool
        {
            // Path::validate only compiles for Space == Robot (see plan.hh); task-space path
            // validity goes through LocalPlanner::validate / resolve() + the ambient robot's
            // validate() instead.
            throw std::runtime_error(
                "path.validate() is not supported for a task-space Path; resolve() each "
                "waypoint and validate the ambient robot's configuration instead");
        }

        static auto result_solved(const PlanningResult &r) -> bool
        {
            return r.solved;
        }

        static void sampler_skip(Sampler &s, std::size_t n)
        {
            for (std::size_t i = 0; i < n; ++i)
            {
                s.next();
            }
        }

        static auto sampler_next(Sampler &s) -> Cfg
        {
            return Input::from(s.next());
        }
    };
}  // namespace vamp::binding
