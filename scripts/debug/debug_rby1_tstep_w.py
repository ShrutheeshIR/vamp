import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner
problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
for w in (5.0, 1.0, 0.3):
    pl = PyPlanner(setup, w); pl.run(3000)
    T = np.array(pl.T); P = np.array(pl.parent); n0 = 1 + sum(1 for i in range(1, len(T)) if pl.parent[i] == i - 1 and i < 12 and False)
    dt = (T - T[P])[5:]
    print(f"weight {w}: nodes {len(T)}, mean dt per explored node {dt.mean():+.4f}, median {np.median(dt):+.4f}", flush=True)
