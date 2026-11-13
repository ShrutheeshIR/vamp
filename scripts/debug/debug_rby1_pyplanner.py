"""A Python copy of the curve planner's outer loop (TaskSpaceRRTC::solve, task_space_rrtc.hh) on top of the C++ pieces:
curve_steer (the constrained local planner's steer, with the reason it ended), curve_goal_step (the goal region's error and
Levenberg-Marquardt step), the halton sampler and a curve parameter from the same van der Corput sequence as CurveRNG.
Differences from the C++ planner: exact nearest neighbour (the C++ tree is approximate, nn_epsilon), brute force.
Records, for every explore and walk, what happened and why."""
import collections
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

M = d.module


def vdc(i):
    """The curve parameter of sample i (CurveRNG: van der Corput base 2, index from 1)."""
    t, scale = 0.0, 0.5
    while i:
        if i & 1:
            t += scale
        scale *= 0.5
        i >>= 1
    return t


class PyPlanner:
    def __init__(self, setup, weight, q_start=None, t_start=0.0, range_=d.RANGE, max_greedy=d.MAX_GREEDY_STEPS,
                 monotone_t=d.MONOTONE_T, stability=d.STABILITY, accept="end_error",
                 qweights=None, sample_box=None, walk_mode="lm", shrink_on=None, min_shrink=0.125):
        self.shrink_on, self.min_shrink = shrink_on, min_shrink   # shrink_on: steer failure name ("trace_continuity") after which the goal step is retried at half size
        self.walk_mode = walk_mode   # "lm": steps toward the end pose in joint space with t pushed to 1 (the C++ goal walk); "advance": advance t along the curve, projecting back, halving the t step on failure
        # qweights: per-joint weights of the metric (default 1); sample_box: (lo, hi) arrays of the box the random samples are drawn in (default: the module's bounds)
        self.qw = np.ones(23) if qweights is None else np.asarray(qweights, float)
        self.box = sample_box
        self.accept = accept   # the walk accepts a step if "end_error": the squared error to the end pose decreases (the C++ rule); "t": t increases
        self.w, self.range, self.max_greedy, self.monotone = weight, range_, max_greedy, monotone_t
        self.cs = setup.curves + ([setup.com] if stability else [])
        self.settings = d.make_settings()
        self.tol = self.settings.tolerance
        q0 = np.asarray(setup.q_start if q_start is None else q_start, np.float32)
        self.Q, self.T, self.parent = [q0], [float(t_start)], [0]
        self.sampler, self.index = M.halton(), 0
        self.explore_log = collections.Counter()   # outcome of each explore
        self.walk_log = collections.Counter()      # why each walk ended
        self.fail_nodes = collections.defaultdict(collections.Counter)  # nearest node -> outcomes
        self.solution = None
        self.history = []                          # (iteration, tree size, max t)

    # --- helpers ---
    def metric(self, q, t, Q, T):
        return np.sqrt(np.sum((self.qw * (Q - q)) ** 2, axis=1) + (self.w * (T - t)) ** 2)

    def steer(self, i, q_target, t_target, distance):
        return M.curve_steer(self.Q[i], self.T[i], np.asarray(q_target, np.float32), float(t_target), float(distance), self.range,
                             d.env, self.cs, self.settings)

    def goal(self, q):
        return M.curve_goal_step(np.asarray(q, np.float32), self.cs, self.settings)

    def chain_monotone(self, t_from, ts):
        for t in ts:
            if t < t_from - 1e-6:
                return False
            t_from = t
        return True

    def add_chain(self, parent, joints, ts):
        for q, t in zip(joints, ts):
            self.Q.append(np.asarray(q, np.float32)); self.T.append(float(t)); self.parent.append(parent)
            parent = len(self.Q) - 1
        return parent

    # --- explore: straight extension of the nearest node toward a random sample ---
    def explore(self):
        self.index += 1
        q_s, t_s = np.asarray(self.sampler.next(), np.float32), vdc(self.index)
        if self.box is not None:                  # redraw the sample in the given box (same Halton point, rescaled)
            lo0, hi0 = np.asarray(M.lower_bounds(), float), np.asarray(M.upper_bounds(), float)
            q_s = (self.box[0] + (q_s - lo0) / (hi0 - lo0) * (self.box[1] - self.box[0])).astype(np.float32)
        Qa, Ta = np.array(self.Q), np.array(self.T)
        dist = self.metric(q_s, t_s, Qa, Ta)
        i = int(np.argmin(dist)); distance = float(dist[i])
        if self.monotone and t_s < self.T[i]:
            t_s = self.T[i]
            distance = float(self.metric(q_s, t_s, Qa[i:i + 1], Ta[i:i + 1])[0])
        status, joints, ts, info = self.steer(i, q_s, t_s, distance)
        if status in (0, 1):
            key = "steer: " + info["failure"] + (" (" + info["projection"]["exit"] + ")" if info["failure"] == "no_lane_projected" else "")
            self.explore_log[key] += 1; self.fail_nodes[i][key] += 1
            return None
        if self.monotone and not self.chain_monotone(self.T[i], ts):
            self.explore_log["chain not monotone in t"] += 1; self.fail_nodes[i]["chain not monotone in t"] += 1
            return None
        if not joints:
            self.explore_log["no waypoints"] += 1
            return None
        self.explore_log["success"] += 1
        return self.add_chain(i, joints, ts)

    # --- walk, mode "advance": extend in t at constant joints (the steer projects the target back onto the curve) ---
    def walk_advance(self, node):
        cur = node
        t = self.T[cur]
        for _ in range(self.max_greedy):
            err, _ = self.goal(self.Q[cur])
            if err <= self.tol:
                self.solution = cur; self.walk_log["reached the goal"] += 1; return
            delta, got = self.range / self.w, False
            while delta > 1e-3 and t < 1.0:
                t_next = min(1.0, t + delta)
                status, joints, ts, info = self.steer(cur, self.Q[cur], t_next, self.w * (t_next - t))
                if status >= 2 and joints and ts[-1] > t + 1e-6 and self.chain_monotone(t, ts):
                    cur = self.add_chain(cur, joints, ts); t = ts[-1]; got = True; break
                delta *= 0.5
            if not got:
                self.walk_log["advance: no t step worked"] += 1; return
        else:
            self.walk_log["step limit"] += 1
        err, _ = self.goal(self.Q[cur])
        if err <= self.tol:
            self.solution = cur; self.walk_log["reached the goal"] += 1

    # --- walk: greedy goal-directed steps from a node ---
    def walk(self, node):
        if self.walk_mode == "advance":
            return self.walk_advance(node)
        cur = node
        q, t = self.Q[cur].astype(np.float64), self.T[cur]
        error, _ = self.goal(q)
        for _ in range(self.max_greedy):
            if error <= self.tol:
                break
            error, proposed = self.goal(q)
            delta = np.asarray(proposed, np.float64) - q
            dt = 1.0 - t
            norm = float(np.sqrt(np.sum((self.qw * delta) ** 2) + (self.w * dt) ** 2))
            if not norm > 1e-9:
                self.walk_log["zero step"] += 1; return
            clamp = min(1.0, self.range / norm)
            q_next, t_next = q + delta * clamp, t + dt * clamp
            status, joints, ts, info = self.steer(cur, q_next, t_next, min(norm, self.range))
            f = 1.0
            while status in (0, 1) and self.shrink_on == info["failure"] and f > self.min_shrink:
                f *= 0.5; c2 = clamp * f
                status, joints, ts, info = self.steer(cur, q + delta * c2, t + dt * c2, min(norm, self.range) * f)
                self.walk_log["shrunk to x%g" % f] += 1
            if status in (0, 1):
                self.walk_log["steer: " + info["failure"]] += 1; return
            if self.monotone and not self.chain_monotone(t, ts):
                self.walk_log["chain not monotone in t"] += 1; return
            q_e, t_e = np.asarray(joints[-1], np.float64), ts[-1]
            e_err, _ = self.goal(q_e)
            if self.accept == "end_error" and not e_err < error:
                self.walk_log["end-pose error did not decrease"] += 1; return
            if self.accept == "t" and not t_e > t + 1e-6:
                self.walk_log["t did not increase"] += 1; return
            cur = self.add_chain(cur, joints, ts)
            q, t, error = q_e, t_e, e_err
        else:
            self.walk_log["step limit"] += 1
        if error <= self.tol:
            self.solution = cur; self.walk_log["reached the goal"] += 1

    def run(self, max_iterations, report=()):
        self.walk(0)
        self.iterations = 0
        for it in range(1, max_iterations + 1):
            self.iterations = it
            if self.solution is not None:
                break
            added = self.explore()
            if added is not None:
                self.walk(added)
            if it in report or it == max_iterations:
                self.history.append((it, len(self.Q), max(self.T)))
        return self.solution is not None


if __name__ == "__main__":
    problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 3000
    report = (1, 3, 10, 30, 100, 300, 1000, 3000, 10000, 20000)
    pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0], accept=sys.argv[2] if len(sys.argv) > 2 else "end_error")
    solved = pl.run(n, report)
    print(f"\nPython planner, {n} iterations: solved={solved}")
    print("  iteration -> tree size, highest t in the tree   (C++ planner: 1->6, 3->8, 10->18, 30->29, 100->32, 300->35, 1000->49, 3000->86, 20000->247)")
    for it, size, tmax in pl.history:
        print(f"  {it:>6} -> {size:>5}   t_max={tmax:.3f}")
    print("\nexplore outcomes:", dict(pl.explore_log.most_common()))
    print("walk stop reasons:", dict(pl.walk_log.most_common()))
    print("nodes that most explores nearest-hit and failed from (node: t, outcomes):")
    for node, c in sorted(pl.fail_nodes.items(), key=lambda kv: -sum(kv[1].values()))[:5]:
        print(f"  node {node}: t={pl.T[node]:.3f}, {dict(c.most_common(3))}")
