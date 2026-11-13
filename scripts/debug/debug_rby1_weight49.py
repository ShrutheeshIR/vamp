"""sid 49: parameter weight against the solve, in the C++ planner (all three) and the Python copy (advance-t walk)."""
import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

sid = int(sys.argv[1]) if len(sys.argv) > 1 else 49
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
w = np.ones(23); w[0] = w[1] = float(np.max((hi - lo)[3:])) / float(hi[0] - lo[0])
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
print(f"sid {sid}: parameter weight -> C++ planner (all three) | Python copy, advance-t walk")
for pw in (5.0, 10.0, 17.0, 25.0, 50.0, 100.0):
    r = d.plan_with(setup, pw)
    pl = PyPlanner(setup, pw, qweights=w, accept="t", walk_mode="advance"); ok = pl.run(3000)
    print(f"  weight {pw:>5g}: C++ solved={r.solved!s:<5} it={int(r.iterations):>5} ms={r.nanoseconds / 1e6:6.0f} | Python solved={ok!s:<5} it={pl.iterations:>5} tree={len(pl.Q):>5}", flush=True)
