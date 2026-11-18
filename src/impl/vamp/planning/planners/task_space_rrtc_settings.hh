#pragma once

#include <cstddef>
#include <vector>

#include <vamp/planning/planners/rrtc_settings.hh>

namespace vamp::planning
{
    // The settings of the task-space RRT-Connect: those of RRTC (the longest extension `range`, which bounds both an exploration
    // step toward a random sample and a goal-directed step toward the region; max_iterations; max_samples; nn_epsilon) and
    // what is specific to planning toward a region. RRTC's settings for the two trees it grows from a start and a goal
    // configuration (dynamic_domain, radius, alpha, min_radius, balance, tree_ratio, start_tree_first) are not used.
    struct TaskSpaceRRTCSettings : RRTCSettings
    {
        // RRTC's dynamic domain, off by default here (RRTCSettings turns it on): a node whose extensions toward samples
        // failed only accepts samples within its own radius (`radius` after the first failure, shrinking by (1 - alpha)
        // per failure down to `min_radius`, growing by (1 + alpha) per success), in the planner's metric.
        TaskSpaceRRTCSettings()
        {
            dynamic_domain = false;
        }

        // Steps taken by the goal-directed walk that follows each exploration step.
        std::size_t max_greedy_steps = 20;

        // Optional, off by default: the longest step of the goal-directed walk, if it should differ from the
        // exploration `range`. 0 uses `range`. Near obstacles a long walk step can jump over the narrow way to the
        // region (the straight segment of a step is checked, not the way between steps), so a smaller walk step
        // follows it more closely.
        float walk_range = 0.;

        // Optional, off by default: interleave IK with planning. On this fraction of the iterations (0 turns it
        // off; 0.01 is every hundredth), a random sample is projected onto the region (a full projection, with the
        // region's own method and iteration limit: a bounded least-squares method and 100+ iterations work well),
        // and a collision-free result becomes the root of a goal tree, up to `ik_pool_size` roots. On the same
        // iterations the goal tree is extended toward a random sample, and new nodes of either tree are connected to
        // the nearest node of the other, as in RRT-Connect. Meeting solves the problem. This helps when the region is
        // hard to walk into, e.g. a pose in a tight space.
        float ik_rate = 0.F;
        std::size_t ik_pool_size = 100;

        // Project every random sample onto the manifold (lp.project) before the nearest-neighbor query and the
        // extension toward it, instead of leaving it for the extension's projection. A sample that does not project
        // costs its iteration. For planning in a Space whose states carry a curve parameter (CurveSpace), where a
        // sample has to be on the curve at its own parameter for "nearest" to mean "nearest on the curve". Only
        // constrained (projecting) local planners project.
        bool project_samples = false;

        // For a Space with a curve parameter t: the weight of t in the metric of the run, in rad per unit of t (the
        // joints' distance is in rad). The nearest-neighbor tree measures distances as sqrt(|dq|^2 + (w dt)^2), and
        // `range` bounds a step in that metric. With the goal-directed walk heading for t = 1, a larger weight makes t
        // advance more slowly per step (about range / w when far from the goal), a smaller one faster. 0 (the default)
        // is 1 / the number of joints: the whole curve costs as much as one joint's rad on average.
        float parameter_weight = 0.;

        // For a Space with a curve parameter t: the weight of t in the nearest-neighbor query alone, in rad per unit of t.
        // Negative (the default) is the same as parameter_weight. The step length and the goal walk always measure with
        // parameter_weight, so a large parameter_weight keeps a step close to the curve while a small nn_parameter_weight lets a
        // random sample pick the node with the most similar joints at any t, not the node nearest in t (with a large weight,
        // samples ahead of the tree all extend its frontier).
        float nn_parameter_weight = -1.F;

        // For a Space with a curve parameter t: if positive, an exploration step is decoupled. The joints move toward the sample by
        // at most `range` (measured in the joints alone, the joint weights but no t), and t moves toward the sample's t by at most
        // this much, instead of every coordinate moving the same fraction of the way (range / distance). 0 (the default) keeps
        // the shared fraction. The goal-directed walk is unchanged.
        float t_step_cap = 0.F;

        // Experiment, off by default (1 keeps every node): the goal-directed walk adds every node it creates to the path
        // structure (so a solution path stays continuous), but only this fraction of them to the nearest-neighbor tree, which
        // is the part `max_samples` bounds. After a walk ends, node k of its n nodes is kept with probability
        // min(1, 2 f k / (n + 1)) (later nodes more likely) and the last is always kept. The buffer holds every node
        // (up to 2 / f x max_samples).
        float walk_keep_fraction = 1.F;

        // Experiment, off by default: every iteration makes two extension attempts toward the same random sample. The first
        // starts at the frontier (the node with the highest t, no nearest-neighbor query), the second at the nearest node
        // as usual (set nn_parameter_weight small for the nearest in the joints alone). Needs t_step_cap > 0 (the decoupled
        // step), since the frontier's distance to the sample is not measured. Each successful attempt is followed by a walk.
        bool frontier_attempt = false;

        // With frontier_attempt: if positive, the first attempt starts at a node drawn by its t instead of always the
        // highest-t node: a t level is drawn with probability proportional to exp(-(t_front - t) / tau) over the non-empty
        // levels (0.005 wide), then a node of that level uniformly. 0 (the default) always takes the frontier node.
        float frontier_tau = 0.F;

        // With frontier_attempt (and frontier_tau = 0): if above 1, the first attempt starts at one of the `frontier_pool`
        // nodes with the highest t, chosen uniformly. 0 or 1: always the highest-t node.
        std::size_t frontier_pool = 0;

        // Debugging, off by default: keep up to this many examples of steers that failed the drift check (every 50th such
        // failure of each kind), each with the node's state, the step's target state and the projected state.
        std::size_t capture_drift = 0;

        // With t_step_cap (the decoupled step), experiment, off by default: the planar x and y of a mobile base (the joints
        // with a metric weight below 0.5) do not move toward the sample in an exploration step: they stay at the node's
        // values and are left out of the step's distance. The projection still moves them as the curve requires.
        bool step_fix_base_xy = false;

        // With t_step_cap, experiment, off by default (0): the planar x and y of a mobile base take a step of their own in an
        // exploration step: a random direction in the plane and a length drawn log-uniformly in [base_step_min, base_step_max]
        // (metres), so most steps are short (about the distance the hands move for the t step) and some are long enough to
        // reorient. These joints are left out of the step's distance, as with step_fix_base_xy.
        float base_step_min = 0.F;
        float base_step_max = 0.F;

        // With t_step_cap, experiment, off by default: the base x and y step with the hands. Their step is the displacement of
        // the centroid of the reference hands' xy between the node's t and the step's t (feed-forward, read from `ref_xy`
        // at the parameters `ref_t`, linearly interpolated) plus an exploration term: the difference between the base-minus-
        // hands offsets of two random nodes, so the base reorients by about as much as the nodes' offsets already differ.
        // Needs ref_t (L values, increasing) and ref_xy (2 L values). These joints are left out of the step's distance.
        bool base_feedforward = false;
        std::vector<float> ref_t;
        std::vector<float> ref_xy;

        // For planning in a CurveSpace (the Python task_space_rrtc_curve): after a solve, insert states until consecutive
        // ones are at most this far apart in the joints (infinity norm, rad), each interpolated in (q, t), projected back
        // onto every constraint of the manifold (the curves of every end-effector, and a center-of-mass constraint if there
        // is one), and every new edge revalidated. If anything fails the planner's path is returned unchanged (the
        // result's `interpolated` is false). 0 (the default) disables it. A linear interpolation of a planned path
        // leaves the curves between its states, by centimetres for a path of few states; this keeps the dense path on them.
        float interpolate_step = 0.;

        // For a Space with a curve parameter t: keep t non-decreasing along every path the planner returns, so a path
        // never retreats along the curve (a scorer that wants the curve followed in order reads a retreat as leaving
        // the tube). An exploration step toward a sample whose t is below its nearest node's is steered toward the
        // sample's joints at the node's t instead (the sample's t is raised to the node's), and an extension whose
        // chain of waypoints still has t decreasing (a segment-form constraint solves for t) is rejected, in the walk
        // too. A node can still move at constant t (the joints change at one point of the curve). On by default, as a
        // path that retreats is rarely wanted; false gives the unconstrained behaviour. It has no effect in the
        // robot's own space.
        bool monotone_t = true;

        // For a Space with a curve parameter t: per-joint weights of the metric (the robot's joints, in order; empty
        // means every weight is 1). The metric becomes sqrt(sum (w_j dq_j)^2 + (parameter_weight dt)^2): the
        // nearest-neighbor tree, the step length (`range`), the goal walk's step and the drift guards of the projection
        // and of the steer all measure in it. It makes joints of different units comparable (a planar base's metres
        // against the arms' radians): the straight lines, and so the direction of a step, do not change, only how far
        // along it a step of `range` goes and what counts as drift. Must have one entry per joint to take effect.
        std::vector<float> joint_weights;

        // For a Space with a curve parameter t: the goal walk accepts a step if it advances t (progress along the
        // curve), instead of if the squared error to the region at the end of the curve decreases. On a curve whose hands
        // first move away from their final pose, the error rises along the curve and the default rule stops the walk
        // there, though every step along the curve is progress. Off by default (the error rule).
        bool walk_by_parameter = false;
    };
}  // namespace vamp::planning
