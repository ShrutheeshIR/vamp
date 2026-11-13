"""Is the path the Python planner finds with accept='t' a real solution? Valid nodes, on the curve at their t, and the whole path passes interpolate_curve_path."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0], accept="t")
print("\nsolved:", pl.run(50), "tree size", len(pl.Q))
i, idx = pl.solution, []
while True:
    idx.append(i)
    if pl.parent[i] == i or i == 0: break
    i = pl.parent[i]
idx = idx[::-1]
Q = [pl.Q[i] for i in idx]; T = [pl.T[i] for i in idx]
print(f"path: {len(Q)} nodes, t from {T[0]:.3f} to {T[-1]:.3f}, t monotone: {all(b >= a - 1e-6 for a, b in zip(T, T[1:]))}")
print("nodes valid in VAMP's model:", sum(bool(d.module.validate(q.astype(np.float32), d.env, True)) for q in Q), "of", len(Q))
err = [d.module.squared_error(q.astype(np.float32), [d.region_at(setup, t)] + ([setup.com] if d.STABILITY else [])) for q, t in zip(Q, T)]
print(f"squared error to the curve at each node's t: max {max(err):.1e}; end pose error at the last node: {pl.goal(Q[-1])[0]:.1e}")
dense = d.module.interpolate_curve_path([q.astype(np.float32) for q in Q], [float(t) for t in T], d.env, pl.cs, pl.settings, d.MAX_KNOT_STEP)
print("interpolate_curve_path (projects and collision-checks every piece):", dense.interpolated)
print(f"largest joint step between consecutive nodes: {max(np.max(np.abs(b - a)) for a, b in zip(Q, Q[1:])):.3f} rad")
