"""From the nodes the Python planner fails from most: success rate of curve_steer toward random samples at several ranges, and toward the walk's next state."""
import collections
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner, vdc

M = d.module
problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0]); pl.run(3000)
nodes = [n for n, c in sorted(pl.fail_nodes.items(), key=lambda kv: -sum(kv[1].values()))[:4]]
path, _ = d.walk_reference(setup, "BoundedLS")
wt = np.array([t for t, _ in path]); wq = np.array([q for _, q in path])
W, SAMPLES = d.PARAMETER_WEIGHTS[0], 200
sam = M.halton(); idx = [0]


def steer(i, q_t, t_t, rng_):
    q, t = pl.Q[i], pl.T[i]
    dist = float(np.sqrt(np.sum((np.asarray(q_t, float) - q) ** 2) + (W * (t_t - t)) ** 2))
    s, j, ts, info = M.curve_steer(q, t, np.asarray(q_t, np.float32), float(t_t), dist, rng_, d.env, pl.cs, pl.settings)
    return s, info["failure"]


for i in nodes:
    q, t = pl.Q[i], pl.T[i]
    k = int(np.argmin(np.abs(wt - t)))
    print(f"\nnode {i}: t={t:.3f}; distance to the walk's configuration at that t: {np.linalg.norm(wq[k] - q):.3f} rad")
    for rng_ in (0.25, 0.1, 0.05, 0.02):
        c = collections.Counter()
        for _ in range(SAMPLES):
            idx[0] += 1
            q_s, t_s = np.asarray(sam.next(), np.float32), max(vdc(idx[0]), t)
            s, f = steer(i, q_s, t_s, rng_)
            c["success" if s >= 2 else f] += 1
        print(f"   random samples, range {rng_:<5}: " + ", ".join(f"{k_} {v / SAMPLES:.0%}" for k_, v in c.most_common(3)))
    for dt in (0.02, 0.1):
        k2 = int(np.argmin(np.abs(wt - min(1.0, t + dt))))
        s, f = steer(i, wq[k2], wt[k2], 0.25)
        print(f"   toward the walk's state at t+{dt} (range 0.25): status {s}, {f}")
    s, f = steer(i, wq[k], t, 0.25)
    print(f"   toward the walk's configuration at the same t (range 0.25): status {s}, {f}")
