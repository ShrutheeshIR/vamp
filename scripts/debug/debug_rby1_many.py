"""Other starts where a baseline passes the 32-pose window and the planner failed (rby1_find.log): the C++ planner as is, and the
Python copy with the two changes (base yaw sampled in start yaw +- pi; base xy weight = largest revolute range / 40 m; xy box stays +-20 m).
Usage: debug_rby1_many.py sid sid ..."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:]))
print(f"sid   start valid   C++ planner as is        Python copy, yaw +-pi + xy weight {max_rev / 40:.2f}")
for sid in [int(a) for a in sys.argv[1:]]:
    try:
        import io, contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
        valid = bool(d.module.validate(setup.q_start.astype(np.float32), d.env, True))
        r = d.plan_with(setup, 5.0)
        cpp = f"solved={r.solved!s:<5} it={int(r.iterations):>5} tree={list(r.size)[0]:>5}"
        yaw0 = float(setup.q_start[2]); blo, bhi = lo.copy(), hi.copy(); blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi
        w = np.ones(23); w[0] = w[1] = max_rev / 40
        pl = PyPlanner(setup, 5.0, qweights=w, sample_box=(blo, bhi)); ok = pl.run(5000)
        py = f"solved={ok!s:<5} it={pl.iterations:>5} tree={len(pl.Q):>5}"
        if ok:
            i, idx = pl.solution, []
            while True:
                idx.append(i)
                if i == 0: break
                i = pl.parent[i]
            Q = [pl.Q[j] for j in idx[::-1]]; T = [pl.T[j] for j in idx[::-1]]
            py += " path valid: " + str(d.module.interpolate_curve_path([q.astype(np.float32) for q in Q], [float(t) for t in T], d.env, pl.cs, pl.settings, d.MAX_KNOT_STEP).interpolated)
        print(f"{sid:>3}   {valid!s:<11}  {cpp}   {py}", flush=True)
    except Exception as e:
        print(f"{sid:>3}   error {type(e).__name__}: {str(e)[:80]}", flush=True)
