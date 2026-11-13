"""sid 49 only. From states on the exact walk's path: a pure-t step (same joints, t + dt) through the C++ steer, at parameter weight 5 and 25, with the steer's
own account of why it ends: the failure, the step length the weight gives it, the drift limit (2 x step) and the drift the projection actually needs."""
import contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

M = d.module
with contextlib.redirect_stdout(io.StringIO()):
    problem = d.load_problem(49, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(M.lower_bounds(), float), np.asarray(M.upper_bounds(), float)
w = np.ones(23); w[0] = w[1] = float(np.max((hi - lo)[3:])) / float(hi[0] - lo[0])
cs = setup.curves + [setup.com]
st = d.make_settings(); st.metric_weights = [float(v) for v in w]          # what the planner hands its projections
path, _ = d.walk_reference(setup, "BoundedLS")
wt = np.array([t for t, _ in path]); wq = np.array([q for _, q in path])
print("sid 49, pure-t steps from states on the exact walk (same joints, t -> t + dt), C++ steer, range 0.25")
print("t       weight  dt     step (=w dt)  result                    drift needed  limit (2 step)  projection exit/iterations")
for t0 in (0.0, 0.3, 0.6, 0.9):
    k = int(np.argmin(np.abs(wt - t0))); q, t = wq[k].astype(np.float32), float(wt[k])
    for pw in (5.0, 25.0):
        for dt in (0.05, 0.01, 0.002):
            s, j, ts, info = M.curve_steer(q, t, q, min(1.0, t + dt), float(pw * dt), d.RANGE, d.env, cs, st)
            pr = info["projection"]
            print(f"{t:.2f}   {pw:>5g}  {dt:<6g} {pw * dt:>9.3f}     {('ok' if s >= 2 else info['failure']):<24} {info['drift']:>10.3f}   {info['drift_limit']:>10.3f}      {pr['exit']}/{pr['iterations']}")
