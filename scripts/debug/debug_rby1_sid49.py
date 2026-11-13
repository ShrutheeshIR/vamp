"""sid 49 with all three changes (goal walk accepts t, base xy metric weight, yaw bounds +-pi from the module): the exact walk reaches the end, the C++
planner fails. The Python copy of the planner: where does the tree stall and why? Usage: debug_rby1_sid49.py [sid] [iterations]"""
import collections, contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

M = d.module
sid = int(sys.argv[1]) if len(sys.argv) > 1 else 49
n_it = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(M.lower_bounds(), float), np.asarray(M.upper_bounds(), float)
w = np.ones(23); w[0] = w[1] = float(np.max((hi - lo)[3:])) / float(hi[0] - lo[0])
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
r = d.plan_with(setup, 5.0)
print(f"sid {sid}: C++ planner with all three: solved={r.solved}, iterations {int(r.iterations)}, tree {list(r.size)}")

path, _ = d.walk_reference(setup, "BoundedLS")
wt = np.array([t for t, _ in path]); wq = np.array([q for _, q in path])
step = np.linalg.norm(np.diff(wq, axis=0), axis=1)
print(f"exact walk reaches t={wt[-1]:.2f}; joint motion per 0.02 step: median {np.median(step):.3f}, max {step.max():.3f} (at t={wt[int(np.argmax(step))]:.2f})")

pl = PyPlanner(setup, 5.0, qweights=w, accept="t")
solved = pl.run(n_it, (30, 100, 300, 1000, 3000, 5000, 10000))
print(f"\nPython copy: solved={solved}, iterations {pl.iterations}")
print("  (iteration, tree size, highest t):", [(i, n, round(t, 3)) for i, n, t in pl.history])
print("  explore outcomes:", dict(pl.explore_log.most_common(5)))
print("  walk stops:", dict(pl.walk_log.most_common(4)))
T = np.array(pl.T)
print("  tree nodes by t: " + ", ".join(f"{a:.1f}-{b:.1f}: {int(((T >= a) & (T < b + (1e-9 if b < 1 else 1))).sum())}" for a, b in [(0, .2), (.2, .4), (.4, .6), (.6, .8), (.8, 1.0)]))
top = [i for i in np.argsort(-T)[:3]]
print("  from the 3 highest-t nodes, curve_steer toward the walk's state at t+0.02 / +0.05 / +0.1:")
for i in top:
    out = []
    for dt in (0.02, 0.05, 0.1):
        k = int(np.argmin(np.abs(wt - min(1.0, pl.T[i] + dt))))
        dist = float(np.sqrt(np.sum((w * (wq[k] - pl.Q[i])) ** 2) + (5.0 * (wt[k] - pl.T[i])) ** 2))
        s, j, ts, info = M.curve_steer(pl.Q[i], pl.T[i], wq[k].astype(np.float32), float(wt[k]), dist, d.RANGE, d.env, pl.cs, pl.settings)
        out.append(f"dt {dt}: " + ("ok" if s >= 2 else info["failure"] + (f"/{info['projection']['exit']}" if info["failure"] == "no_lane_projected" else "")))
    k0 = int(np.argmin(np.abs(wt - pl.T[i])))
    print(f"    node {i} t={pl.T[i]:.3f} (|q - walk(t)| {np.linalg.norm(pl.Q[i] - wq[k0]):.2f}): " + "; ".join(out))
# the frontier: what collides at the endpoints of failing steps
fails = collections.Counter(); pairs = collections.Counter()
sam = M.halton(); st = d.make_settings(); rg = np.random.default_rng(0)
for i in top:
    q, t = pl.Q[i].astype(float), pl.T[i]
    for _ in range(150):
        q_s = np.asarray(sam.next(), float); t_s = max(rg.uniform(0, 1), t)
        dist = np.sqrt(np.sum((w * (q_s - q)) ** 2) + (5.0 * (t_s - t)) ** 2); k = min(1.0, d.RANGE / dist)
        q1, t1 = q + (q_s - q) * k, t + (t_s - t) * k
        try:
            qp = np.asarray(M.project(np.clip(q1, lo, hi).astype(np.float32), [d.region_at(setup, t1), setup.com], st), float)
        except ValueError:
            fails["projection fails"] += 1; continue
        if M.validate(qp.astype(np.float32), d.env, True): fails["valid"] += 1
        else:
            fails["invalid endpoint"] += 1
            for p in d.pair_names(setup, qp): pairs[p] += 1
print("  steps toward random samples from those nodes:", dict(fails), "; colliding pairs:", dict(pairs.most_common(3)))
