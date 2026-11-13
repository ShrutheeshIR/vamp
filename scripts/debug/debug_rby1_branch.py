"""How far, and in which joints, do the tree's early nodes (the planner's first goal-directed walk from the start) sit from the walk's path at the same t?"""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0]); pl.walk(0)          # only the first goal walk from the start
path, _ = d.walk_reference(setup, "BoundedLS")
wt = np.array([t for t, _ in path]); wq = np.array([q for _, q in path])
names = [n.replace("left_arm", "L").replace("right_arm", "R").replace("torso", "T").replace("base/", "") for n in model.joint_names]
print(f"\nfirst goal walk from the start: {len(pl.Q)} nodes")
print("node   t     |q - walk(t)|   largest differing joints (rad / m)")
for i in range(len(pl.Q)):
    k = int(np.argmin(np.abs(wt - pl.T[i])))
    dq = np.asarray(pl.Q[i], float) - wq[k]
    top = np.argsort(-np.abs(dq))[:3]
    print(f"{i:>4}  {pl.T[i]:.3f}  {np.linalg.norm(dq):8.3f}      " + ", ".join(f"{names[j]} {dq[j]:+.2f}" for j in top))
