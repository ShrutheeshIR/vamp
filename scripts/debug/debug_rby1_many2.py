"""The 13 starts, Python copy, 5000 iterations: (a) yaw +-pi + xy weight (old goal-walk rule); (b) goal-walk rule 'accept if t increases' only;
(c) all three. Prints solved/iterations and whether the path passes interpolate_curve_path."""
import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:]))


def run(setup, yawxy, accept):
    blo, bhi, w = lo.copy(), hi.copy(), np.ones(23)
    if yawxy:
        yaw0 = float(setup.q_start[2]); blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi; w[0] = w[1] = max_rev / 40
    pl = PyPlanner(setup, 5.0, qweights=w, sample_box=(blo, bhi), accept=accept); ok = pl.run(5000)
    if not ok:
        return "FAIL"
    i, idx = pl.solution, []
    while True:
        idx.append(i)
        if i == 0: break
        i = pl.parent[i]
    Q = [pl.Q[j] for j in idx[::-1]]; T = [pl.T[j] for j in idx[::-1]]
    good = d.module.interpolate_curve_path([q.astype(np.float32) for q in Q], [float(t) for t in T], d.env, pl.cs, pl.settings, d.MAX_KNOT_STEP).interpolated
    return f"{pl.iterations:>5}{'' if good else ' (path NOT valid)'}"


print("sid   (a) yaw+xy, old rule   (b) accept-t rule only   (c) all three   [iterations to solve; FAIL = not solved in 5000]")
for sid in [int(a) for a in sys.argv[1:]]:
    with contextlib.redirect_stdout(io.StringIO()):
        problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
    print(f"{sid:>3}   {run(setup, True, 'end_error'):<22} {run(setup, False, 't'):<23} {run(setup, True, 't')}", flush=True)
