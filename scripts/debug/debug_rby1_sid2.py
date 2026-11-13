"""sid 39, the stall near t = 0.75. (1) How fast do the joints move along the walk's path there? (2) From the planner's highest-t nodes, what does
steering toward the walk's next states do? (3) The planner with the goal-walk rule 'accept if t increases'."""
import collections, contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

M = d.module
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(39, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(M.lower_bounds(), float), np.asarray(M.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:])); yaw0 = float(setup.q_start[2])
blo, bhi = lo.copy(), hi.copy(); blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi
w = np.ones(23); w[0] = w[1] = max_rev / 40
names = [n.replace("left_arm", "L").replace("right_arm", "R").replace("torso", "T").replace("base/", "") for n in model.joint_names]

path, _ = d.walk_reference(setup, "BoundedLS")
wt = np.array([t for t, _ in path]); wq = np.array([q for _, q in path])
step = np.linalg.norm(np.diff(wq, axis=0), axis=1)
print("(1) joint motion per walk step of 0.02 in t (rad/m), median %.3f; the steps near t=0.6-0.9:" % np.median(step))
for i in range(len(step)):
    if 0.6 <= wt[i] < 0.9:
        j = np.argsort(-np.abs(wq[i + 1] - wq[i]))[:2]
        print(f"   t {wt[i]:.2f}->{wt[i + 1]:.2f}: {step[i]:.3f}   (" + ", ".join(f"{names[k]} {wq[i + 1][k] - wq[i][k]:+.2f}" for k in j) + ")")

pl = PyPlanner(setup, 5.0, qweights=w, sample_box=(blo, bhi)); pl.run(1500)
T = np.array(pl.T); top = np.argsort(-T)[:3]
print(f"\n(2) planner (1500 iterations): highest t {T.max():.3f}. From its 3 highest-t nodes, curve_steer toward the walk's state at t+0.02 / t+0.05 / t+0.1:")
for i in top:
    out = []
    for dt in (0.02, 0.05, 0.1):
        k = int(np.argmin(np.abs(wt - min(1.0, pl.T[i] + dt))))
        dist = float(np.sqrt(np.sum((w * (wq[k] - pl.Q[i])) ** 2) + (5.0 * (wt[k] - pl.T[i])) ** 2))
        s, j, ts, info = M.curve_steer(pl.Q[i], pl.T[i], wq[k].astype(np.float32), float(wt[k]), dist, d.RANGE, d.env, pl.cs, pl.settings)
        out.append(f"dt {dt}: " + ("ok" if s >= 2 else info["failure"] + (f"/{info['projection']['exit']}" if info["failure"] == "no_lane_projected" else "")))
    k0 = int(np.argmin(np.abs(wt - pl.T[i])))
    print(f"   node {i} t={pl.T[i]:.3f} (|q - walk(t)| {np.linalg.norm(pl.Q[i] - wq[k0]):.2f}): " + "; ".join(out))

pt = PyPlanner(setup, 5.0, qweights=w, sample_box=(blo, bhi), accept="t"); ok = pt.run(5000)
print(f"\n(3) goal-walk rule 'accept if t increases': solved={ok}, iterations {pt.iterations}, tree {len(pt.Q)}, highest t {max(pt.T):.3f}")
