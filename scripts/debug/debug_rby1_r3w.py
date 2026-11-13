"""sid 49: parameter weight = c x (total R3 travel of both hands over the window, m)."""
import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
sid = int(sys.argv[1]) if len(sys.argv) > 1 else 49
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
P = problem.pos
L = float(sum(np.sum(np.linalg.norm(np.diff(P[:, e], axis=0), axis=1)) for e in range(2)))
print(f"sid {sid}: total R3 travel of both hands {L:.1f} m")
for c in (0.5, 0.8, 1.0, 1.2, 1.5, 2.0):
    r = d.plan_with(setup, c * L)
    print(f"  weight = {c:g} x {L:.1f} = {c * L:5.1f}: solved={r.solved!s:<5} iterations={int(r.iterations):>5} ms={r.nanoseconds / 1e6:.0f}", flush=True)
