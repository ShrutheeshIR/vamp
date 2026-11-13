"""Diagnose one start (N = 31, stride 6) with the two changes in the Python copy (yaw +-pi around the start's, base xy weight = largest revolute
range / 40 m): does the exact walk reach the end, what does the end-pose error do along it, and where/why does the Python planner stall.
Usage: debug_rby1_sid.py SID [iterations]"""
import collections, contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

sid, n_it = int(sys.argv[1]), int(sys.argv[2]) if len(sys.argv) > 2 else 5000
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:])); yaw0 = float(setup.q_start[2])
print(f"sid {sid}: start valid in VAMP's model: {bool(d.module.validate(setup.q_start.astype(np.float32), d.env, True))}")

# exact walk and the end-pose error along it
for method in ("BoundedLS", "InnerLM"):
    path, rej = d.walk_reference(setup, method)
    print(f"walk {method}: reaches t={path[-1][0]:.2f}" + (f", STALLS: {dict(collections.Counter(w for w, _ in rej))}" if rej else " (end)"))
path, rej = d.walk_reference(setup, "BoundedLS")
end = d.region_at(setup, 1.0)
err = np.array([d.module.squared_error(q.astype(np.float32), [end]) for _, q in path])
rises = [i for i in range(1, len(err)) if err[i] > err[i - 1]]
print(f"end-pose error along the walk's path: rises at {len(rises)} of {len(err) - 1} steps; first rise at t={path[rises[0]][0]:.2f}" if rises else "end-pose error falls all the way")

# the Python planner with both changes
blo, bhi = lo.copy(), hi.copy(); blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi
w = np.ones(23); w[0] = w[1] = max_rev / 40
pl = PyPlanner(setup, 5.0, qweights=w, sample_box=(blo, bhi))
solved = pl.run(n_it, (30, 100, 300, 1000, 3000, 5000, 10000))
print(f"\nPython planner (yaw +-pi, xy weight): solved={solved}, iterations {pl.iterations}")
print("  (iteration, tree size, highest t):", [(i, n, round(t, 3)) for i, n, t in pl.history])
print("  explore outcomes:", dict(pl.explore_log.most_common(5)))
print("  walk stops:", dict(pl.walk_log.most_common(4)))
T = np.array(pl.T)
print(f"  tree nodes by t: " + ", ".join(f"{a:.1f}-{b:.1f}: {int(((T >= a) & (T < b + (1e-9 if b < 1 else 1))).sum())}" for a, b in [(0, .2), (.2, .4), (.4, .6), (.6, .8), (.8, 1.0)]))
top = [n for n, c in sorted(pl.fail_nodes.items(), key=lambda kv: -sum(kv[1].values()))[:4]]
for i in top:
    k = int(np.argmin(np.abs(np.array([t for t, _ in path]) - pl.T[i])))
    print(f"  node {i}: t={pl.T[i]:.3f}, failed explores {dict(pl.fail_nodes[i].most_common(2))}, |q - walk(t)| = {np.linalg.norm(pl.Q[i] - path[k][1]):.2f}")
