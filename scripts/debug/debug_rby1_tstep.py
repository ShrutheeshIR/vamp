"""Old setup (accept='end_error'): how much does t advance per node the random exploration adds?"""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0]); pl.run(3000)
T = np.array(pl.T); P = np.array(pl.parent)
dt = (T - T[P])[5:]                      # nodes after the first goal walk's 5 (all added by explore, one per success unless noted)
print(f"\n{len(pl.Q)} nodes, {pl.explore_log['success']} successful explores of 3000 iterations")
print(f"t of the first goal walk's last node: {T[4]:.3f}; highest t in the tree after 3000 iterations: {T.max():.3f}")
print(f"t advance per node added by explore: mean {dt.mean():+.4f}, median {np.median(dt):+.4f}, largest {dt.max():+.4f}; "
      f"zero (fixed-t steps) {np.mean(np.abs(dt) < 1e-9):.0%}, positive {np.mean(dt > 1e-9):.0%}")
Q = np.array(pl.Q); step = np.linalg.norm(Q - Q[P], axis=1)[5:]
print(f"joint distance per node: median {np.median(step):.3f} rad")
