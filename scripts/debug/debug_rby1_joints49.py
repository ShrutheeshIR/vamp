import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(49, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
path, _ = d.walk_reference(setup, "BoundedLS")
Q = np.array([q for _, q in path]); tot = np.sum(np.abs(np.diff(Q, axis=0)), axis=0)
names = [n.replace("left_arm", "L").replace("right_arm", "R").replace("torso", "T").replace("base/", "") for n in model.joint_names]
print(f"sid 49: total joint motion along the exact walk (t 0 -> 1), sum of |dq| per joint; all joints together {tot.sum():.1f}")
for j in np.argsort(-tot)[:6]:
    print(f"   {names[j]:<10} {tot[j]:6.2f}   (start {Q[0][j]:+.2f} -> end {Q[-1][j]:+.2f})")
P = problem.pos
print("hand travel over the window (m), left / right:", [round(float(np.sum(np.linalg.norm(np.diff(P[:, e], axis=0), axis=1))), 2) for e in range(2)])
