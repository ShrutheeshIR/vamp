"""sid 49 (and others): the Python planner with the goal walk replaced by 'advance t along the curve' (project back, halve the step on failure)."""
import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
w = np.ones(23); w[0] = w[1] = float(np.max((hi - lo)[3:])) / float(hi[0] - lo[0])
print("sid   LM-direction walk (all three)    advance-t walk (same weights)   [solved, iterations, tree]")
for sid in [int(a) for a in sys.argv[1:]]:
    with contextlib.redirect_stdout(io.StringIO()):
        problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
    row = []
    for mode in ("lm", "advance"):
        pl = PyPlanner(setup, 5.0, qweights=w, accept="t", walk_mode=mode); ok = pl.run(5000)
        row.append(f"{ok!s:<5} {pl.iterations:>5} {len(pl.Q):>5}")
    print(f"{sid:>3}   {row[0]:<30}   {row[1]}", flush=True)
