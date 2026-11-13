"""Smoke test of the new bindings: curve_steer from the start toward random samples; tally of info['failure'] and the planner unchanged."""
import collections, sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
W, R = d.PARAMETER_WEIGHTS[0], d.RANGE
rng = np.random.default_rng(0)
q0 = setup.q_start.astype(np.float32)
cs = setup.curves + [setup.com]
c = collections.Counter(); exits = collections.Counter()
for _ in range(300):
    q_s, t_s = rng.uniform(model.q_lo, model.q_hi).astype(np.float32), float(rng.uniform(0, 1))
    dist = float(np.sqrt(np.sum((q_s - q0) ** 2) + (W * t_s) ** 2))
    status, joints, ts, info = d.module.curve_steer(q0, 0.0, q_s, t_s, dist, R, d.env, cs, d.make_settings())
    c[(status, info["failure"])] += 1
    exits[(info["failure"], info["projection"]["exit"])] += 1
print("(status, failure):", dict(c)); print("(failure, steer-projection exit):", dict(exits))
print("one info:", info)
r = d.plan_with(setup, 5.0)
print("planner unchanged: solved", r.solved, "iterations", int(r.iterations), "tree", list(r.size))
