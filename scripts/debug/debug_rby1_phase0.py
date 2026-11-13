"""Phase 0 of the planner debugging on the RB-Y1 problem of debug_rby1.py (start 26, N = 31).
 1. End-pose error along the walk's path: does it fall with t? (The planner's goal step accepts a step only if the squared
    error to the end pose, the curve at t = 1, decreases.)
 2. Planner started from the walk's configuration at t0 (start_t = t0), for t0 from 0.9 down to 0: where does it start failing?
Run in the container: python3 -u scripts/debug_rby1_phase0.py"""
import sys
import time
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
path, rejected = d.walk_reference(setup, "BoundedLS")
print(f"\nwalk (BoundedLS) reaches t={path[-1][0]:.2f}; {len(path)} states")

# --- 1. end-pose error along the path ---
end = d.region_at(setup, 1.0)
err_end = np.array([d.module.squared_error(q.astype(np.float32), [end]) for _, q in path])
err_cur = np.array([d.module.squared_error(q.astype(np.float32), [d.region_at(setup, t)]) for t, q in path])
print("\n1. squared error to the end pose (curve at t=1) along the walk's path; '^' = larger than at the previous state")
print("   t     error to end pose   error to curve at t")
for i, (t, _) in enumerate(path):
    print(f"  {t:.2f}   {err_end[i]:12.6f} {'^' if i and err_end[i] > err_end[i - 1] else ' '}   {err_cur[i]:.1e}")
up = [i for i in range(1, len(path)) if err_end[i] > err_end[i - 1]]
print(f"   steps where the end-pose error rises: {len(up)} of {len(path) - 1}"
      + (f"; largest rise {max(err_end[i] - err_end[i - 1] for i in up):.4f} at t={path[max(up, key=lambda i: err_end[i] - err_end[i - 1])][0]:.2f}" if up else ""))

# --- 2. planner from the walk's state at t0 ---
w = d.PARAMETER_WEIGHTS[0]
print(f"\n2. planner (parameter_weight={w:g}, range={d.RANGE}) started from the walk's state at t0, start_t=t0")
print("   t0    solved  iterations  ms")
by_t = {round(t, 2): q for t, q in path}
for t0 in (0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.0):
    t_key = min(by_t, key=lambda x: abs(x - t0))
    r = d.plan_with(setup, w, q_start=by_t[t_key], start_t=t_key)
    print(f"  {t_key:.2f}   {r.solved!s:<6}  {int(r.iterations):>9}  {r.nanoseconds / 1e6:8.0f}", flush=True)
