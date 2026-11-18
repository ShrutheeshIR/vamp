#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <unordered_set>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <vamp/collision/environment.hh>
#include <vamp/planning/constraints/manifold/constraint_set.hh>
#include <vamp/planning/constraints/utils.hh>
#include <vamp/planning/local_planner.hh>
#include <vamp/planning/nn/nn.hh>
#include <vamp/planning/plan.hh>
#include <vamp/planning/validate.hh>
#include <vamp/planning/planners/task_space_rrtc_settings.hh>
#include <vamp/random/rng.hh>
#include <vamp/planning/space.hh>
#include <vamp/utils.hh>
#include <vamp/vector.hh>

namespace vamp::planning
{
    // RRT-Connect toward a task-space goal region instead of a goal configuration. No goal
    // configuration is ever computed (no IK): the region is a ConstraintSet (e.g. a TSR) and
    // a single tree is grown from the start.
    //
    // Each iteration extends the nearest node toward a random sample with the local planner
    // (as RRTC does), then "connects": from the node just added, a greedy walk takes
    // Levenberg-Marquardt steps on the constraint error, each clamped to `range`, adding a
    // node per step, until a step fails to lower the error, hits a joint limit or a
    // collision, or the region is reached. This plays the role of RRTC's connect step toward
    // the goal tree, with the region's error gradient in place of the straight line to a goal
    // configuration.
    //
    // The planner succeeds when a node satisfies the region (error within the set's
    // tolerance). Edges are straight lines checked by the local planner; only the
    // goal-directed steps look at the constraint.
    //
    // The tree's nodes are states of `Space` (the robot's configurations when it is its own Space). A Space with an
    // extra coordinate (CurveSpace's curve parameter t) is planned in as a whole: samples, extensions and nodes carry
    // it, while the region and collision checking see the ambient configuration (the first Robot::dimension
    // coordinates). The metric of the run is the joints' with t weighted, sqrt(|dq|^2 + (w dt)^2), w =
    // settings.parameter_weight (by default 1 / the number of joints): the nearest-neighbor tree measures in it, and `range` bounds a step in it. The region
    // is held at the end of the curve (t = 1), and a goal-directed step heads for (the region's LM point, t = 1),
    // clamped to `range` in that metric, so the joints and t advance together. With a projecting local planner the
    // step is then projected onto the manifold at its new t (which the local planner holds fixed while it projects the
    // joints).
    template <typename Robot, std::size_t rake, std::size_t resolution, typename Space = Robot>
    struct TaskSpaceRRTC
    {
        using Traits = space::SpaceTraits<Robot, Space>;
        using Configuration = typename Traits::State;
        static constexpr auto dimension = Traits::dimension;
        static constexpr auto ambient_dimension = Robot::dimension;
        using RNG = typename vamp::rng::RNG<Robot, Space>;
        using Region = constraint::ConstraintSet<Robot, rake>;
        using Block = typename Robot::template ConfigurationBlock<rake>;

        template <typename LocalPlanner = UnconstrainedLocalPlanner<Robot, rake, resolution>>
        inline static auto solve(
            const Configuration &start,
            const Region &region,
            const collision::Environment<FloatVector<rake>> &environment,
            const TaskSpaceRRTCSettings &settings,
            typename RNG::Ptr rng,
            const LocalPlanner &lp = LocalPlanner()) noexcept -> PlanningResult<Robot, Space>
        {
            size_t extend_counter_failed = 0;
            size_t extend_counter_succeeded = 0;
            size_t explore_calls = 0;
            std::array<std::size_t, 12> explore_failures{};  // diagnostics: by SteerFailure (index 11: ok)
            std::array<std::size_t, 12> walk_failures{};
            size_t monotone_rejects = 0;
            size_t walk_steps = 0;
            std::array<std::size_t, 9> dt_ahead{};   // diagnostics: |t(sample) - t(nearest node)|, sample ahead of the node; log bins <1e-4,3e-4,1e-3,3e-3,0.01,0.03,0.1,0.3,>=0.3
            std::array<std::size_t, 9> dt_behind{};  // same, sample behind the node
            size_t sample_project_failures = 0;
            std::array<std::size_t, 21> pick_bins{};  // diagnostics: t of the node picked as nearest, 0.01 wide bins up to 0.2

            PlanningResult<Robot, Space> result;
            const auto start_time = std::chrono::steady_clock::now();

            // the weight of the curve parameter in the metric of the run: the setting, or by default 1 / the joints
            const float parameter_weight = settings.parameter_weight > 0.F
                                               ? settings.parameter_weight
                                               : 1.F / static_cast<float>(ambient_dimension);

            NN<Space> tree;
            tree.reserve(settings.max_samples);
            tree.set_epsilon(settings.nn_epsilon);
            const bool thinning = settings.walk_keep_fraction > 0.F and settings.walk_keep_fraction < 1.F;
            const std::size_t capacity =
                thinning ? settings.max_samples * static_cast<std::size_t>(std::ceil(2.F / settings.walk_keep_fraction))
                         : settings.max_samples;
            std::vector<float> radii(capacity + 1, std::numeric_limits<float>::max());
            std::array<std::size_t, 4> proj_exit{};   // diagnostics: how the projection ended in steers that failed with NoLaneProjected (Converged, MaxIterations, Stalled, Drifted)
            std::array<std::size_t, 9> proj_err{};    // and sqrt(best squared error) of those: log bins <1e-3,3e-3,0.01,0.03,0.1,0.3,1,3,>=3
            std::array<std::size_t, 2> drift_kind{};   // diagnostics: Drifted projections that tripped on the move from the previous iterate / from the start
            std::array<std::size_t, 10> t_all{};      // node t (0.1 bins) of every steer
            std::array<std::size_t, 10> t_noproj{};   // node t of steers that failed with NoLaneProjected
            std::array<std::size_t, 9> base_noproj{}; // largest move (m) of the base-xy joints (metric weight < 0.5) in the step: NoLaneProjected steers, log bins <0.01,0.03,0.1,0.3,1,3,10,30,>=30
            std::array<std::size_t, 9> base_ok{};     // the same for steers that worked
            std::array<std::size_t, 6> drift_thr{};    // threshold of Drifted projections, bins <0.1,0.3,1,2,3,>=3 (weighted joint distance)
            std::array<std::size_t, 7> drift_ratio_prev{};   // (move of the last iteration) / threshold at the trip: bins <1.1,<1.5,<2,<5,<20,<100,>=100
            std::array<std::size_t, 2> att_calls{}, att_ok{}, att_walks{}, att_walks_past{};  // by attempt kind: 0 nearest-neighbor, 1 frontier
            std::size_t cur_mode = 0;
            static constexpr std::size_t state_dim_cap = Traits::extra > 0 ? ambient_dimension + 1 : ambient_dimension;
            std::vector<std::size_t> drift_examples;   // captured drift failures: per example 4 scalars + node + target + projected states, floats as (x + 100) * 1e6
            std::size_t drift_seen = 0;
            std::array<std::array<std::size_t, 12>, 2> fail_mode{};   // explore outcomes by SteerFailure (11 ok), per attempt kind
            std::array<std::array<std::size_t, 5>, 2> drift_step{};   // step length at Drift failures, per attempt kind: <1/32,<0.1,<0.3,<range,>=range
            static constexpr std::size_t n_t_bins = 200;
            std::vector<std::vector<std::size_t>> t_bins(n_t_bins);   // node indices by t (width 1 / n_t_bins), for the frontier draw
            std::uint32_t front_state = 88172645U;
            std::vector<std::pair<float, std::size_t>> top_nodes;   // the nodes with the highest t, sorted by t descending (size <= frontier_pool)
            std::size_t frontier_index = 0;       // the node with the highest t so far
            std::size_t walks_total = 0, walks_past = 0, walks_cap = 0;  // walks run, ended with t past the frontier at their start, used all max_greedy_steps attempts
            std::array<std::size_t, 8> walk_gap{}, walk_gap_past{};      // (frontier t at start) - (start node t), same bins as pick_gap; and those that passed the frontier
            std::array<std::size_t, 8> pick_gap{};    // diagnostics: (highest t in the tree) - (t of the picked node); log bins <1e-3,3e-3,.01,.03,.1,.3,.6,>=.6
            float t_front = 0.F;                      // the highest t of any node so far
            std::array<std::size_t, 7> drift_ratio_start{};  // (distance from the start) / threshold, same bins
            std::size_t proj_iters = 0;               // sum of their projection iterations
            std::size_t dd_rejects = 0;  // diagnostics: samples dropped because they lie outside the nearest node's domain
            const bool separate_nn_weight = Traits::extra > 0 and settings.nn_parameter_weight >= 0.F;
            tree.set_parameter_weight(separate_nn_weight ? settings.nn_parameter_weight : parameter_weight);

            // per-joint weights of the metric (settings.joint_weights): only in a Space with a curve parameter
            const bool weighted_joints =
                Traits::extra > 0 and settings.joint_weights.size() == static_cast<std::size_t>(ambient_dimension);
            std::array<float, ambient_dimension> joint_weight;
            std::array<float, ambient_dimension> joint_weight2;
            for (auto j = 0U; j < ambient_dimension; ++j)
            {
                joint_weight[j] = (weighted_joints) ? settings.joint_weights[j] : 1.F;
                joint_weight2[j] = joint_weight[j] * joint_weight[j];
            }

            if constexpr (Traits::extra > 0)
            {
                if (weighted_joints)
                {
                    tree.set_joint_weights(joint_weight.data());
                }
            }

            if constexpr (Traits::extra > 0)
            {
                // the goal region is the constraint at the end of the curve
                region.set_parameter(FloatVector<rake, 1>::fill(1.F));
            }

            auto buffer = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                capacity * Configuration::num_scalars_rounded);
            auto parents =
                vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(capacity);

            const auto buffer_index = [&buffer](std::size_t index) -> float *
            { return buffer.get() + index * Configuration::num_scalars_rounded; };

            const float tolerance = region.settings().tolerance;
            std::size_t iter = 0;
            const float walk_range = (settings.walk_range > 0.F) ? settings.walk_range : settings.range;

            // diagnostics: why a steer failed (the constraint local planner's SteerFailure; 11: it did not)
            const auto failure_index = [&](const auto &ext) -> std::size_t
            {
                if (ext.status != SteerStatus::Trapped and ext.status != SteerStatus::Rejected)
                {
                    return 11;
                }

                if constexpr (LocalPlanner::projecting)
                {
                    return static_cast<std::size_t>(lp.diagnostics().failure);
                }
                else
                {
                    return 0;
                }
            };

            std::size_t free_index = 0;
            std::size_t tree_count = 0;       // nodes in the nearest-neighbor tree (all nodes unless the walk thins them)
            bool deferring = false;           // while a walk runs under thinning, its nodes wait for the tree decision
            std::vector<std::size_t> pending;
            std::uint32_t thin_state = 2463534242U;
            std::uint32_t base_state = 1812433253U;
            const auto xorshift01 = [](std::uint32_t &st)
            {
                st ^= st << 13;
                st ^= st >> 17;
                st ^= st << 5;
                return static_cast<float>(st >> 8) / 16777216.F;
            };
            const auto has_room = [&]() { return free_index < capacity and tree_count < settings.max_samples; };
            const auto add_node = [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (free_index >= capacity or (not deferring and tree_count >= settings.max_samples))
                {
                    return std::nullopt;
                }

                float *ptr = buffer_index(free_index);
                c.to_array(ptr);
                if (deferring)
                {
                    pending.push_back(free_index);
                }
                else
                {
                    tree.insert(free_index, ptr);
                    tree_count++;
                }

                parents[free_index] = (free_index == 0) ? 0 : parent;
                if constexpr (Traits::extra > 0)
                {
                    t_bins[std::min<std::size_t>(n_t_bins - 1, static_cast<std::size_t>(std::max(0.F, ptr[ambient_dimension]) * static_cast<float>(n_t_bins)))].push_back(free_index);
                    if (settings.frontier_pool > 1)
                    {
                        const float tn = ptr[ambient_dimension];
                        if (top_nodes.size() < settings.frontier_pool or tn > top_nodes.back().first)
                        {
                            top_nodes.emplace_back(tn, free_index);
                            std::size_t i = top_nodes.size() - 1;
                            while (i > 0 and top_nodes[i].first > top_nodes[i - 1].first)
                            {
                                std::swap(top_nodes[i], top_nodes[i - 1]);
                                --i;
                            }

                            if (top_nodes.size() > settings.frontier_pool) { top_nodes.pop_back(); }
                        }
                    }

                    if (ptr[ambient_dimension] > t_front or free_index == 0)
                    {
                        t_front = ptr[ambient_dimension];
                        frontier_index = free_index;
                    }
                }

                return free_index++;
            };

            using Lane = std::array<float, dimension>;
            const auto load = [&buffer_index](std::size_t index) -> Lane
            {
                Lane lane;
                const float *ptr = buffer_index(index);
                for (auto j = 0U; j < dimension; ++j)
                {
                    lane[j] = ptr[j];
                }

                return lane;
            };

            const auto lane_of = [](const Configuration &c) -> Lane
            {
                Lane lane;
                const auto array = c.to_array();
                for (auto j = 0U; j < dimension; ++j)
                {
                    lane[j] = array[j];
                }

                return lane;
            };

            // The constraint kernels evaluate `rake` lanes at once; the walk has a single
            // configuration, so every lane holds a copy of its joints.
            const auto broadcast = [](const Lane &q) -> Block
            {
                Block block;
                for (auto j = 0U; j < ambient_dimension; ++j)
                {
                    block[j] = FloatVector<rake>::fill(q[j]);
                }

                return block;
            };

            std::optional<std::size_t> solution;

            // monotone_t: whether t stays non-decreasing from `t_from` along a chain of waypoints
            [[maybe_unused]] const auto monotone_chain = [&](float t_from, const auto &waypoints) -> bool
            {
                if constexpr (Traits::extra > 0)
                {
                    for (const auto &waypoint : waypoints)
                    {
                        const float t = waypoint.to_array()[ambient_dimension];
                        if (t < t_from - 1e-6F)
                        {
                            return false;
                        }

                        t_from = t;
                    }
                }

                return true;
            };

            // Greedy walk from `node` toward the region: one clamped LM step per added node.
            const auto walk_impl = [&](std::size_t node)
            {
                std::size_t current = node;
                Lane q = load(current);
                Block block = broadcast(q);
                float error = region.squared_error(block)[{0, 0}];

                for (std::size_t step = 0; step < settings.max_greedy_steps; ++step)
                {
                    if (error <= tolerance)
                    {
                        break;
                    }

                    // The kernel cache holds the Jacobian at `block`: take the LM step from it.
                    Block proposed = block;
                    region.step_in_place(proposed);
                    // std::cout << "Step " << step << ": error = " << error << ", proposed = " << proposed << std::endl;

                    Lane next = q;
                    float norm2 = 0.F;
                    for (auto j = 0U; j < ambient_dimension; ++j)
                    {
                        next[j] = proposed[{j, 0}] - q[j];
                        norm2 += next[j] * next[j] * joint_weight2[j];
                    }

                    // The step heads for (the region's LM point, the end of the curve): t is carried toward 1 along with
                    // the joints, and counts in the length of the step with its weight
                    [[maybe_unused]] float parameter_delta = 0.F;
                    if constexpr (Traits::extra > 0)
                    {
                        parameter_delta = 1.F - q[ambient_dimension];
                        const float weighted = parameter_weight * parameter_delta;
                        norm2 += weighted * weighted;
                    }

                    const float norm = std::sqrt(norm2);
                    if (not(norm > 1e-9F))
                    {
                        return;
                    }

                    const float clamp = std::min(1.F, walk_range / norm);
                    for (auto j = 0U; j < ambient_dimension; ++j)
                    {
                        next[j] = q[j] + next[j] * clamp;
                    }

                    if constexpr (Traits::extra > 0)
                    {
                        // the same fraction of the way to the end of the curve
                        next[ambient_dimension] = q[ambient_dimension] + parameter_delta * clamp;
                    }

                    if constexpr (LocalPlanner::projecting and Traits::extra > 0)
                    {
                        // On a curve manifold the LM point is only a target: it heads straight for the region and
                        // leaves the manifold. lp.steer takes the step toward it and projects onto the manifold, as
                        // every extension of a constrained RRT does (with the continuity and collision checks), and
                        // returns the chain of on-manifold waypoints. The walk then judges the chain's endpoint.
                        const Configuration target(next);
                        const float step_length = std::min(norm, settings.range);
                        const auto extension = lp.steer(
                            Configuration(buffer_index(current)), target, step_length, settings.range, true, environment);
                        walk_steps++;
                        walk_failures[failure_index(extension)]++;

                        if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                        {
                            return;
                        }

                        if (settings.monotone_t and not monotone_chain(q[ambient_dimension], extension.waypoints))
                        {
                            return;
                        }

                        const Lane endpoint = lane_of(extension.endpoint());
                        const Block endpoint_block = broadcast(endpoint);
                        const float endpoint_error = region.squared_error(endpoint_block)[{0, 0}];
                        bool progressed = endpoint_error < error;
                        if (settings.walk_by_parameter)
                        {
                            // progress along the curve: t advances
                            progressed = endpoint[ambient_dimension] > q[ambient_dimension] + 1e-6F;
                        }

                        if (not progressed)
                        {
                            return;
                        }

                        const auto [last, truncated] = insert_chain<Robot, Space>(
                            extension.waypoints,
                            current,
                            [&](const Configuration &c, std::size_t parent) { return add_node(c, parent); });
                        if (truncated or last == current)
                        {
                            return;
                        }

                        current = last;
                        q = endpoint;
                        block = endpoint_block;  // the region's Jacobian cache now holds this block, for the next step
                        error = endpoint_error;
                        continue;
                    }

                    Block candidate = broadcast(next);
                    constraint::renormalize_so3<Robot, rake>(candidate);
                    const float next_error = region.squared_error(candidate)[{0, 0}];

                    if (not(next_error < error))
                    {
                        return;
                    }

                    const Configuration c(next);
                    if (not Traits::in_bounds(c))
                    {
                        return;
                    }

                    if (not lp.validate(Configuration(buffer_index(current)), c, environment))
                    {
                        return;
                    }

                    const auto added = add_node(c, current);
                    if (not added)
                    {
                        return;
                    }

                    current = *added;
                    q = next;
                    block = candidate;
                    error = next_error;
                }

                if (error <= tolerance)
                {
                    solution = current;
                }
            };

            const auto walk = [&](std::size_t node)
            {
                const float front0 = t_front;
                const std::size_t steps0 = walk_steps;
                const float gap = front0 - buffer_index(node)[ambient_dimension];
                if (thinning)
                {
                    deferring = true;
                    pending.clear();
                }

                walk_impl(node);
                if (thinning)
                {
                    deferring = false;
                    const std::size_t n = pending.size();
                    for (std::size_t k = 0; k < n; ++k)
                    {
                        thin_state ^= thin_state << 13;
                        thin_state ^= thin_state >> 17;
                        thin_state ^= thin_state << 5;
                        const float u = static_cast<float>(thin_state >> 8) / 16777216.F;
                        const float p = std::min(1.F, 2.F * settings.walk_keep_fraction * static_cast<float>(k + 1) / static_cast<float>(n + 1));
                        if ((k + 1 == n or u < p) and tree_count < settings.max_samples)
                        {
                            tree.insert(pending[k], buffer_index(pending[k]));
                            tree_count++;
                        }
                    }
                }

                walks_total++;
                static constexpr std::array<float, 7> wedges{1e-3F, 3e-3F, 0.01F, 0.03F, 0.1F, 0.3F, 0.6F};
                std::size_t wb = 7;
                for (std::size_t e = 0; e < 7; ++e) { if (gap < wedges[e]) { wb = e; break; } }
                walk_gap[wb]++;
                att_walks[cur_mode]++;
                if (t_front > front0 + 1e-6F) { walks_past++; walk_gap_past[wb]++; att_walks_past[cur_mode]++; }
                if (walk_steps - steps0 >= settings.max_greedy_steps) { walks_cap++; }
            };

            // Straight-line extension of the nearest node toward a random sample; returns the
            // node added, if any.
            const auto sample_frontier = [&]() -> std::size_t
            {
                if (settings.frontier_tau <= 0.F)
                {
                    if (settings.frontier_pool > 1 and not top_nodes.empty())
                    {
                        thin_state ^= thin_state << 13;
                        thin_state ^= thin_state >> 17;
                        thin_state ^= thin_state << 5;
                        return top_nodes[(thin_state >> 8) % top_nodes.size()].second;
                    }

                    return frontier_index;
                }

                const float tau = settings.frontier_tau;
                const float width = 1.F / static_cast<float>(n_t_bins);
                const std::size_t top = std::min<std::size_t>(n_t_bins - 1, static_cast<std::size_t>(std::max(0.F, t_front) * static_cast<float>(n_t_bins)));
                const auto level_weight = [&](std::size_t b) -> float
                {
                    if (t_bins[b].empty()) { return 0.F; }
                    const float tb = (b == top) ? t_front : (static_cast<float>(b) + 0.5F) * width;
                    const float delta = t_front - tb;
                    return (delta > 9.F * tau) ? 0.F : std::exp(-delta / tau);
                };
                const auto next01 = [&]()
                {
                    front_state ^= front_state << 13;
                    front_state ^= front_state >> 17;
                    front_state ^= front_state << 5;
                    return static_cast<float>(front_state >> 8) / 16777216.F;
                };

                float total = 0.F;
                for (std::size_t b = top + 1; b-- > 0;)
                {
                    const float w = level_weight(b);
                    if (w == 0.F and static_cast<float>(top - b) * width > 9.F * tau) { break; }
                    total += w;
                }

                float u = next01() * total;
                std::size_t chosen = top;
                for (std::size_t b = top + 1; b-- > 0;)
                {
                    const float w = level_weight(b);
                    if (w > 0.F) { chosen = b; if (u < w) { break; } u -= w; }
                    if (static_cast<float>(top - b) * width > 9.F * tau) { break; }
                }

                const auto &level = t_bins[chosen];
                return level[static_cast<std::size_t>(next01() * static_cast<float>(level.size())) % level.size()];
            };

            using SampleType = std::decay_t<decltype(rng->next())>;
            std::optional<SampleType> shared_sample;
            const auto explore = [&](std::size_t mode) -> std::optional<std::size_t>
            {
                explore_calls++;
                cur_mode = mode;
                const bool reuse = (mode == 0 and shared_sample.has_value());
                auto sample = reuse ? *shared_sample : rng->next();
                if (mode == 0)
                {
                    shared_sample.reset();
                }

                if constexpr (LocalPlanner::projecting)
                {
                    // A sample on the manifold (at its own parameter), so the nearest node is the nearest on it
                    if (settings.project_samples and not lp.project(sample))
                    {
                        sample_project_failures++;
                        return std::nullopt;
                    }
                }

                if (mode == 1)
                {
                    shared_sample = sample;
                }

                typename Traits::StateBuffer sample_array;
                sample.to_array(sample_array.data());

                std::size_t nearest_index = (mode == 1) ? sample_frontier() : frontier_index;
                float nearest_distance = 0.F;
                if (mode == 0)
                {
                    const auto nearest = tree.nearest(sample_array.data());
                    if (not nearest)
                    {
                        return std::nullopt;
                    }

                    nearest_index = nearest->first;
                    nearest_distance = nearest->second;
                }

                if (settings.dynamic_domain and radii[nearest_index] < nearest_distance)
                {
                    dd_rejects++;
                    return std::nullopt;
                }

                if constexpr (Traits::extra > 0)
                {
                    const float pick_t = buffer_index(nearest_index)[ambient_dimension];
                    pick_bins[std::min<std::size_t>(20, static_cast<std::size_t>(std::max(0.F, pick_t) * 100.F))]++;
                    {
                        const float gap = t_front - pick_t;
                        static constexpr std::array<float, 7> gedges{1e-3F, 3e-3F, 0.01F, 0.03F, 0.1F, 0.3F, 0.6F};
                        std::size_t gb = 7;
                        for (std::size_t e = 0; e < 7; ++e) { if (gap < gedges[e]) { gb = e; break; } }
                        pick_gap[gb]++;
                    }
                    {
                        const float dts = sample_array[ambient_dimension] - pick_t;
                        const float a = std::fabs(dts);
                        static constexpr std::array<float, 8> edges{1e-4F, 3e-4F, 1e-3F, 3e-3F, 0.01F, 0.03F, 0.1F, 0.3F};
                        std::size_t bin = 8;
                        for (std::size_t e = 0; e < 8; ++e) { if (a < edges[e]) { bin = e; break; } }
                        (dts >= 0.F ? dt_ahead : dt_behind)[bin]++;
                    }
                }

                auto target = sample;
                float target_distance = nearest_distance;
                if constexpr (Traits::extra > 0)
                {
                    // the tree measured with its own weight of t: the step is measured with the run's
                    const auto step_distance = [&](const auto &to)
                    {
                        const auto node_array = Configuration(buffer_index(nearest_index)).to_array();
                        const auto to_array = to.to_array();
                        float d2 = 0.F;
                        for (auto j = 0U; j < ambient_dimension; ++j)
                        {
                            const float diff = to_array[j] - node_array[j];
                            d2 += diff * diff * joint_weight2[j];
                        }

                        const float dt = parameter_weight * (to_array[ambient_dimension] - node_array[ambient_dimension]);
                        return std::sqrt(d2 + dt * dt);
                    };

                    if (separate_nn_weight)
                    {
                        target_distance = step_distance(sample);
                    }

                    // monotone_t: a sample behind the nearest node in t is steered toward at the node's t
                    if (settings.monotone_t)
                    {
                        const float t_nearest = buffer_index(nearest_index)[ambient_dimension];
                        if (Space::parameter(sample) < t_nearest)
                        {
                            target = Space::make_state(Space::to_ambient(sample), t_nearest);
                            if (separate_nn_weight)
                            {
                                target_distance = step_distance(target);
                            }
                            else if (weighted_joints)
                            {
                                const auto node_array = Configuration(buffer_index(nearest_index)).to_array();
                                const auto target_array = target.to_array();
                                float d2 = 0.F;
                                for (auto j = 0U; j < ambient_dimension; ++j)
                                {
                                    const float diff = target_array[j] - node_array[j];
                                    d2 += diff * diff * joint_weight2[j];
                                }

                                const float dt = parameter_weight * (target_array[ambient_dimension] - node_array[ambient_dimension]);
                                target_distance = std::sqrt(d2 + dt * dt);
                            }
                            else
                            {
                                target_distance = Traits::distance(Configuration(buffer_index(nearest_index)), target);
                            }
                        }
                    }
                }

                if constexpr (Traits::extra > 0)
                {
                    if (settings.t_step_cap > 0.F)
                    {
                        // decoupled step: the joints move by at most `range` toward the sample (joint metric alone), t by at most t_step_cap
                        const auto node_array = Configuration(buffer_index(nearest_index)).to_array();
                        auto target_array = target.to_array();
                        const bool own_base_step = settings.base_step_max > 0.F;
                        const bool feedforward = settings.base_feedforward and settings.ref_t.size() >= 2 and
                                                 settings.ref_xy.size() == 2 * settings.ref_t.size();
                        const auto fixed = [&](std::size_t j)
                        { return (settings.step_fix_base_xy or own_base_step or feedforward) and joint_weight[j] < 0.5F; };
                        float d2 = 0.F;
                        for (auto j = 0U; j < ambient_dimension; ++j)
                        {
                            if (fixed(j)) { continue; }
                            const float diff = target_array[j] - node_array[j];
                            d2 += diff * diff * joint_weight2[j];
                        }

                        const float dq = std::sqrt(d2);
                        const float fraction = (dq > settings.range) ? settings.range / dq : 1.F;
                        const float dt = target_array[ambient_dimension] - node_array[ambient_dimension];
                        const float t_move = std::max(-settings.t_step_cap, std::min(settings.t_step_cap, dt));
                        for (auto j = 0U; j < ambient_dimension; ++j)
                        {
                            target_array[j] = fixed(j) ? node_array[j] : node_array[j] + (target_array[j] - node_array[j]) * fraction;
                        }

                        if (feedforward)
                        {
                            // hands' xy centroid at t, interpolated along the reference
                            const auto centroid = [&](float t, float &cx, float &cy)
                            {
                                const auto &rt = settings.ref_t;
                                const auto &rxy = settings.ref_xy;
                                const std::size_t n = rt.size();
                                std::size_t hi = std::upper_bound(rt.begin(), rt.end(), t) - rt.begin();
                                hi = std::min(std::max<std::size_t>(hi, 1), n - 1);
                                const std::size_t lo = hi - 1;
                                const float span = rt[hi] - rt[lo];
                                const float u = span > 1e-9F ? std::min(1.F, std::max(0.F, (t - rt[lo]) / span)) : 0.F;
                                cx = rxy[2 * lo] + u * (rxy[2 * hi] - rxy[2 * lo]);
                                cy = rxy[2 * lo + 1] + u * (rxy[2 * hi + 1] - rxy[2 * lo + 1]);
                            };
                            std::array<std::size_t, 2> bj{0, 0};
                            std::size_t seen = 0;
                            for (auto j = 0U; j < ambient_dimension and seen < 2; ++j)
                            {
                                if (joint_weight[j] < 0.5F) { bj[seen++] = j; }
                            }

                            const float t_node = node_array[ambient_dimension];
                            float c0x, c0y, c1x, c1y;
                            centroid(t_node, c0x, c0y);
                            centroid(t_node + t_move, c1x, c1y);
                            float ex = 0.F, ey = 0.F;
                            if (free_index >= 2 and seen == 2)
                            {
                                const auto offset = [&](std::size_t i, float &ox, float &oy)
                                {
                                    const float *node = buffer_index(i);
                                    float cx, cy;
                                    centroid(node[ambient_dimension], cx, cy);
                                    ox = node[bj[0]] - cx;
                                    oy = node[bj[1]] - cy;
                                };
                                float ax, ay, bx, by;
                                offset(static_cast<std::size_t>(xorshift01(base_state) * static_cast<float>(free_index)) % free_index, ax, ay);
                                offset(static_cast<std::size_t>(xorshift01(base_state) * static_cast<float>(free_index)) % free_index, bx, by);
                                ex = ax - bx;
                                ey = ay - by;
                            }

                            if (seen == 2)
                            {
                                target_array[bj[0]] = node_array[bj[0]] + (c1x - c0x) + ex;
                                target_array[bj[1]] = node_array[bj[1]] + (c1y - c0y) + ey;
                            }
                        }

                        if (own_base_step)
                        {
                            // a step of the base of its own: random direction, length log-uniform in [base_step_min, base_step_max]
                            const float lo_len = std::max(settings.base_step_min, 1e-4F);
                            const float len = lo_len * std::pow(settings.base_step_max / lo_len, xorshift01(base_state));
                            const float angle = 6.2831853F * xorshift01(base_state);
                            std::size_t seen = 0;
                            for (auto j = 0U; j < ambient_dimension and seen < 2; ++j)
                            {
                                if (joint_weight[j] < 0.5F)
                                {
                                    target_array[j] = node_array[j] + len * (seen == 0 ? std::cos(angle) : std::sin(angle));
                                    seen++;
                                }
                            }
                        }

                        target_array[ambient_dimension] = node_array[ambient_dimension] + t_move;
                        target = Configuration(target_array);
                        target_distance = std::min(dq, settings.range);
                    }
                }

                const auto extension = lp.steer(
                    Configuration(buffer_index(nearest_index)),
                    target,
                    target_distance,
                    settings.range,
                    true,
                    environment);
                explore_failures[failure_index(extension)]++;
                fail_mode[cur_mode][failure_index(extension)]++;
                if constexpr (LocalPlanner::projecting)
                {
                    if (failure_index(extension) == 3)
                    {
                        if (settings.capture_drift > 0 and (drift_seen++ % 50) == 0 and
                            drift_examples.size() < settings.capture_drift * (4 + 3 * state_dim_cap))
                        {
                            const auto enc = [](float x) { return static_cast<std::size_t>((x + 100.F) * 1e6F); };
                            const auto node_arr = Configuration(buffer_index(nearest_index)).to_array();
                            const auto targ_arr = target.to_array();
                            const auto &dg = lp.diagnostics();
                            drift_examples.push_back(enc(static_cast<float>(cur_mode)));
                            drift_examples.push_back(enc(dg.drift));
                            drift_examples.push_back(enc(dg.step_length));
                            drift_examples.push_back(enc(static_cast<float>(dg.projection.iterations)));
                            for (std::size_t j = 0; j < state_dim_cap; ++j) { drift_examples.push_back(enc(node_arr[j])); }
                            for (std::size_t j = 0; j < state_dim_cap; ++j) { drift_examples.push_back(enc(targ_arr[j])); }
                            for (std::size_t j = 0; j < state_dim_cap; ++j) { drift_examples.push_back(enc(dg.endpoint[j])); }
                        }

                        const float sl = lp.diagnostics().step_length;
                        drift_step[cur_mode][(sl < 0.03125F) ? 0 : (sl < 0.1F) ? 1 : (sl < 0.3F) ? 2 : (sl < settings.range * 0.999F) ? 3 : 4]++;
                    }
                }
                if constexpr (Traits::extra > 0)
                {
                    const auto na = Configuration(buffer_index(nearest_index)).to_array();
                    const auto ta = target.to_array();
                    const float sc = (target_distance > settings.range) ? settings.range / target_distance : 1.F;
                    float disp = 0.F;
                    for (auto j = 0U; j < ambient_dimension; ++j)
                    {
                        if (joint_weight[j] < 0.5F) { disp = std::max(disp, std::fabs(ta[j] - na[j]) * sc); }
                    }

                    static constexpr std::array<float, 8> bedges{0.01F, 0.03F, 0.1F, 0.3F, 1.F, 3.F, 10.F, 30.F};
                    std::size_t bb = 8;
                    for (std::size_t k = 0; k < 8; ++k) { if (disp < bedges[k]) { bb = k; break; } }
                    const auto tb = std::min<std::size_t>(9, static_cast<std::size_t>(std::max(0.F, na[ambient_dimension]) * 10.F));
                    t_all[tb]++;
                    const bool failed = extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected;
                    if (not failed) { base_ok[bb]++; }
                    if constexpr (LocalPlanner::projecting)
                    {
                        if (failed and static_cast<std::size_t>(lp.diagnostics().failure) == 2)
                        {
                            base_noproj[bb]++;
                            t_noproj[tb]++;
                        }
                    }
                }

                if constexpr (LocalPlanner::projecting)
                {
                    if ((extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected) and
                        static_cast<std::size_t>(lp.diagnostics().failure) == 2)
                    {
                        const auto &pd = lp.diagnostics().projection;
                        proj_exit[std::min<std::size_t>(3, static_cast<std::size_t>(pd.exit))]++;
                        proj_iters += pd.iterations;
                        if (pd.exit == constraint::ProjectionExit::Drifted) { drift_kind[pd.drift_from_start ? 1 : 0]++; }
                        if (pd.exit == constraint::ProjectionExit::Drifted)
                        {
                            static constexpr std::array<float, 5> tedges{0.1F, 0.3F, 1.F, 2.F, 3.F};
                            std::size_t tb2 = 5;
                            for (std::size_t k = 0; k < 5; ++k) { if (pd.drift_threshold < tedges[k]) { tb2 = k; break; } }
                            drift_thr[tb2]++;
                            static constexpr std::array<float, 6> redges{1.1F, 1.5F, 2.F, 5.F, 20.F, 100.F};
                            const float th = std::max(pd.drift_threshold, 1e-9F);
                            std::size_t rb = 6;
                            for (std::size_t k = 0; k < 6; ++k) { if (pd.drift_move_prev / th < redges[k]) { rb = k; break; } }
                            drift_ratio_prev[rb]++;
                            rb = 6;
                            for (std::size_t k = 0; k < 6; ++k) { if (pd.drift_move_start / th < redges[k]) { rb = k; break; } }
                            drift_ratio_start[rb]++;
                        }
                        static constexpr std::array<float, 8> pedges{1e-3F, 3e-3F, 0.01F, 0.03F, 0.1F, 0.3F, 1.F, 3.F};
                        const float e = std::sqrt(std::max(0.F, pd.best_error));
                        std::size_t b = 8;
                        for (std::size_t k = 0; k < 8; ++k) { if (e < pedges[k]) { b = k; break; } }
                        proj_err[b]++;
                    }
                }


                if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                {
                    if (settings.dynamic_domain)
                    {
                        radii[nearest_index] = (radii[nearest_index] == std::numeric_limits<float>::max())
                                                   ? settings.radius
                                                   : std::max(radii[nearest_index] * (1.F - settings.alpha), settings.min_radius);
                    }

                    extend_counter_failed++;
                    // std::cout << extend_counter_failed << ": Steer failed: " << static_cast<int>(extension.status) << " for config "
                            //   << Configuration(buffer_index(nearest_index)) << " toward " << sample
                            //   << std::endl;
                    return std::nullopt;
                }

                if constexpr (Traits::extra > 0)
                {
                    if (settings.monotone_t and
                        not monotone_chain(buffer_index(nearest_index)[ambient_dimension], extension.waypoints))
                    {
                        monotone_rejects++;
                        return std::nullopt;
                    }
                }

                extend_counter_succeeded++;
                if (settings.dynamic_domain and radii[nearest_index] != std::numeric_limits<float>::max())
                {
                    radii[nearest_index] *= (1.F + settings.alpha);
                }

                // std::cout << extend_counter_succeeded << ": Steer succeeded: " << static_cast<int>(extension.status) << " for config "
                        //   << Configuration(buffer_index(nearest_index)) << " toward " << sample
                        //   << std::endl;

                const auto [last, truncated] = insert_chain<Robot, Space>(
                    extension.waypoints,
                    nearest_index,
                    [&](const Configuration &c, std::size_t parent) { return add_node(c, parent); });
                if (truncated or last == nearest_index)
                {
                    return std::nullopt;
                }

                return last;
            };


            // Optional (settings.ik_rate > 0): a second tree grown from goal configurations found by projecting
            // random samples onto the region. It is grown like the other tree of RRT-Connect: the start tree
            // extends toward a sample, the goal tree extends toward a sample on the IK iterations, and each new node
            // of one tree is connected to the nearest node of the other. Joining the two solves the problem.
            // The interleaved IK goal tree is for planning in the robot's own space: the region of a curve is held at its end
            const bool ik_on = Traits::extra == 0 and settings.ik_rate > 0.F;
            const std::size_t goal_capacity = ik_on ? settings.max_samples : 1;
            NN<Space> goal_tree;
            goal_tree.reserve(goal_capacity);
            goal_tree.set_epsilon(settings.nn_epsilon);
            auto goal_buffer = vamp::utils::buffer_alloc<float, FloatVectorAlignment>(
                goal_capacity * Configuration::num_scalars_rounded);
            auto goal_parents = vamp::utils::buffer_alloc<std::size_t, FloatVectorAlignment>(goal_capacity);
            std::size_t goal_free = 0;
            std::size_t goal_roots = 0;
            std::size_t goal_join = 0;
            bool joined = false;
            float ik_credit = 0.F;
            const auto goal_at = [&goal_buffer](std::size_t index) -> float *
            { return goal_buffer.get() + index * Configuration::num_scalars_rounded; };

            const auto add_goal_node = [&](const Configuration &c, std::size_t parent) -> std::optional<std::size_t>
            {
                if (goal_free >= goal_capacity)
                {
                    return std::nullopt;
                }

                float *ptr = goal_at(goal_free);
                c.to_array(ptr);
                goal_tree.insert(goal_free, ptr);
                goal_parents[goal_free] = parent;
                return goal_free++;
            };

            // Extends the start tree from `from_index` toward `target` until it is reached or blocked, returning the
            // last node of the start tree if the target was reached.
            const auto connect_start_to = [&](std::size_t from_index,
                                              const Configuration &target,
                                              float distance) -> std::optional<std::size_t>
            {
                const std::size_t n_extensions = std::ceil(lp.connect_slack * distance / settings.range);
                Configuration prior(buffer_index(from_index));
                std::size_t prior_index = from_index;
                for (std::size_t i = 0; i < n_extensions and has_room(); ++i)
                {
                    const float remaining = Traits::distance(prior, target);
                    const bool final_step = (i == n_extensions - 1) or (remaining <= settings.range);
                    const auto step = lp.steer(
                        prior,
                        target,
                        remaining,
                        final_step ? std::numeric_limits<float>::max() : settings.range,
                        true,
                        environment);

                    if (step.status == SteerStatus::Trapped or step.status == SteerStatus::Rejected)
                    {
                        return std::nullopt;
                    }

                    const auto [step_index, step_truncated] = insert_chain<Robot, Space>(
                        step.waypoints,
                        prior_index,
                        [&](const Configuration &c, std::size_t parent) { return add_node(c, parent); });
                    if (step_truncated or step_index == prior_index)
                    {
                        return std::nullopt;
                    }

                    if (step.status == SteerStatus::Reached)
                    {
                        return step_index;
                    }

                    prior = step.endpoint();
                    prior_index = step_index;
                }

                return std::nullopt;
            };

            // Tries to join the start tree to goal node `g`, from the nearest start-tree node.
            const auto join_goal_node = [&](std::size_t g)
            {
                const Configuration target(goal_at(g));
                typename Traits::StateBuffer array;
                target.to_array(array.data());
                if (const auto nearest = tree.nearest(array.data()))
                {
                    if (const auto reached = connect_start_to(nearest->first, target, nearest->second))
                    {
                        solution = *reached;
                        goal_join = g;
                        joined = true;
                    }
                }
            };

            // Tries to join the new start-tree node `n` to the nearest goal-tree node.
            const auto join_start_node = [&](std::size_t n)
            {
                typename Traits::StateBuffer array;
                Configuration(buffer_index(n)).to_array(array.data());
                if (const auto nearest = goal_tree.nearest(array.data()))
                {
                    const Configuration target(goal_at(nearest->first));
                    if (const auto reached = connect_start_to(n, target, nearest->second))
                    {
                        solution = *reached;
                        goal_join = nearest->first;
                        joined = true;
                    }
                }
            };

            // One IK iteration: find a goal root if the budget allows, then extend the goal tree toward a sample
            // and try to join its new node to the start tree.
            const auto ik_iteration = [&]()
            {
                if constexpr (Traits::extra == 0)
                {
                    auto seed = rng->next();
                    if (goal_roots < settings.ik_pool_size and region.project(seed) and Robot::in_bounds(seed) and
                        validate_configuration<Robot, rake>(seed, environment))
                    {
                        if (const auto root = add_goal_node(seed, goal_free))
                        {
                            ++goal_roots;
                            goal_parents[*root] = *root;
                            join_goal_node(*root);
                            return;
                        }
                    }
                }

                if (goal_free == 0)
                {
                    return;
                }

                const auto sample = rng->next();
                typename Traits::StateBuffer sample_array;
                sample.to_array(sample_array.data());
                const auto nearest = goal_tree.nearest(sample_array.data());
                const auto extension = lp.steer(
                    Configuration(goal_at(nearest->first)),
                    sample,
                    nearest->second,
                    settings.range,
                    false,
                    environment);

                if (extension.status == SteerStatus::Trapped or extension.status == SteerStatus::Rejected)
                {
                    return;
                }

                const auto [last, truncated] = insert_chain<Robot, Space>(
                    extension.waypoints,
                    nearest->first,
                    [&](const Configuration &c, std::size_t parent) { return add_goal_node(c, parent); });
                if (truncated or last == nearest->first)
                {
                    return;
                }

                join_goal_node(last);
            };

            add_node(start, 0);
            walk(0);

            while (not solution and iter++ < settings.max_iterations and has_room())
            {
                if (settings.frontier_attempt)
                {
                    att_calls[1]++;
                    if (const auto front = explore(1))
                    {
                        att_ok[1]++;
                        walk(*front);
                        if (not solution and goal_free > 0)
                        {
                            join_start_node(*front);
                        }
                    }
                }

                att_calls[0]++;
                if (not solution and has_room())
                {
                    if (const auto added = explore(0))
                    {
                        att_ok[0]++;
                        walk(*added);
                        if (not solution and goal_free > 0)
                        {
                            join_start_node(*added);
                        }
                    }
                }

                if (ik_on and not solution)
                {
                    ik_credit += settings.ik_rate;
                    if (ik_credit >= 1.F)
                    {
                        ik_credit -= 1.F;
                        ik_iteration();
                    }
                }
            }

            if (solution)
            {
                auto current = *solution;
                result.path.emplace_back(buffer_index(current));
                while (parents[current] != current)
                {
                    const auto parent = parents[current];
                    result.path.emplace_back(buffer_index(parent));
                    result.cost += Traits::distance(
                        result.path[result.path.size() - 1], result.path[result.path.size() - 2]);
                    current = parent;
                }

                std::reverse(result.path.begin(), result.path.end());

                // Joined the goal tree: continue from the joined goal node down to its root.
                if (joined)
                {
                    auto g = goal_join;
                    while (goal_parents[g] != g)
                    {
                        g = goal_parents[g];
                        result.path.emplace_back(goal_at(g));
                        result.cost += Traits::distance(
                            result.path[result.path.size() - 1], result.path[result.path.size() - 2]);
                    }
                }

                result.solved = true;
            }

            result.nanoseconds = vamp::utils::get_elapsed_nanoseconds(start_time);
            result.iterations = iter;
            result.size.emplace_back(tree.size());

            // diagnostics: successful and failed extensions toward samples, and (for a curve Space) how many nodes lie in each
            // twentieth of the curve (t in [0, 1])
            result.size.emplace_back(extend_counter_succeeded);
            result.size.emplace_back(extend_counter_failed);
            result.size.emplace_back(explore_calls);
            result.size.emplace_back(sample_project_failures);
            for (const auto b : pick_bins)
            {
                result.size.emplace_back(b);
            }
            if constexpr (Traits::extra > 0)
            {
                std::array<std::size_t, 20> bins{};
                for (std::size_t i = 0; i < free_index; ++i)
                {
                    const float t = buffer_index(i)[ambient_dimension];
                    bins[std::min<std::size_t>(19, static_cast<std::size_t>(std::max(0.F, t) * 20.F))]++;
                }

                for (const auto b : bins)
                {
                    result.size.emplace_back(b);
                }

                // diagnostics for the RB-Y1 (torso joints 3 to 8 of the ambient configuration): for each 0.05 of t up to 0.4,
                // the number of distinct cells (0.3 rad grid) the nodes' torso values fall in, and each torso joint's min and
                // max, as (value + 10) * 1e5
                constexpr std::size_t first_torso = 3;
                constexpr std::size_t n_torso = 6;
                if constexpr (ambient_dimension >= first_torso + n_torso)
                {
                    for (std::size_t band = 0; band < 8; ++band)
                    {
                        std::unordered_set<std::uint64_t> cells;
                        std::array<float, n_torso> lo;
                        std::array<float, n_torso> hi;
                        lo.fill(1e9F);
                        hi.fill(-1e9F);
                        for (std::size_t i = 0; i < free_index; ++i)
                        {
                            const float *node = buffer_index(i);
                            const float t = node[ambient_dimension];
                            if (t < 0.05F * static_cast<float>(band) or t >= 0.05F * static_cast<float>(band + 1))
                            {
                                continue;
                            }

                            std::uint64_t key = 1469598103934665603ULL;
                            for (std::size_t j = 0; j < n_torso; ++j)
                            {
                                const float v = node[first_torso + j];
                                lo[j] = std::min(lo[j], v);
                                hi[j] = std::max(hi[j], v);
                                key = (key ^ static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / 0.3F)) + 1000)) *
                                      1099511628211ULL;
                            }

                            cells.insert(key);
                        }

                        result.size.emplace_back(cells.size());
                        for (std::size_t j = 0; j < n_torso; ++j)
                        {
                            result.size.emplace_back(
                                lo[j] > hi[j] ? 0 : static_cast<std::size_t>((lo[j] + 10.F) * 1e5F));
                            result.size.emplace_back(
                                lo[j] > hi[j] ? 0 : static_cast<std::size_t>((hi[j] + 10.F) * 1e5F));
                        }
                    }
                }
            }

            for (const auto c : explore_failures)
            {
                result.size.emplace_back(c);
            }

            result.size.emplace_back(monotone_rejects);
            for (const auto c : walk_failures)
            {
                result.size.emplace_back(c);
            }

            result.size.emplace_back(walk_steps);
            for (const auto c : dt_ahead) { result.size.emplace_back(c); }
            for (const auto c : dt_behind) { result.size.emplace_back(c); }
            result.size.emplace_back(dd_rejects);
            for (const auto c : proj_exit) { result.size.emplace_back(c); }
            for (const auto c : proj_err) { result.size.emplace_back(c); }
            result.size.emplace_back(proj_iters);
            for (const auto c : drift_kind) { result.size.emplace_back(c); }
            for (const auto c : t_all) { result.size.emplace_back(c); }
            for (const auto c : t_noproj) { result.size.emplace_back(c); }
            for (const auto c : base_noproj) { result.size.emplace_back(c); }
            for (const auto c : base_ok) { result.size.emplace_back(c); }
            for (const auto c : drift_thr) { result.size.emplace_back(c); }
            for (const auto c : drift_ratio_prev) { result.size.emplace_back(c); }
            for (const auto c : drift_ratio_start) { result.size.emplace_back(c); }
            for (const auto c : pick_gap) { result.size.emplace_back(c); }
            result.size.emplace_back(static_cast<std::size_t>(t_front * 1e6F));
            result.size.emplace_back(walks_total);
            result.size.emplace_back(walks_past);
            result.size.emplace_back(walks_cap);
            for (const auto c : walk_gap) { result.size.emplace_back(c); }
            for (const auto c : walk_gap_past) { result.size.emplace_back(c); }
            result.size.emplace_back(tree_count);
            for (const auto c : att_calls) { result.size.emplace_back(c); }
            for (const auto c : att_ok) { result.size.emplace_back(c); }
            for (const auto c : att_walks) { result.size.emplace_back(c); }
            for (const auto c : att_walks_past) { result.size.emplace_back(c); }
            for (const auto &m : fail_mode) { for (const auto c : m) { result.size.emplace_back(c); } }
            for (const auto &m : drift_step) { for (const auto c : m) { result.size.emplace_back(c); } }
            result.size.emplace_back(state_dim_cap);
            result.size.emplace_back(drift_examples.size());
            for (const auto c : drift_examples) { result.size.emplace_back(c); }

            return result;
        }
    };
}  // namespace vamp::planning
