"""Which pairs collide at the endpoints of the failing extensions? Runs the Python planner, then from the nodes that failed most,
repeats the extension step (toward fresh Halton samples, range 0.25, projected onto the curve at the new t) and tallies the
VAMP collision pairs at the endpoints. (The step is copied in Python; the steer's lane perturbations are not.)"""
import collections
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner, vdc

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0]); pl.run(3000)
top = [n for n, c in sorted(pl.fail_nodes.items(), key=lambda kv: -sum(kv[1].values()))[:5]]
st, W, R = d.make_settings(), d.PARAMETER_WEIGHTS[0], d.RANGE
sam = d.module.halton(); idx = 0
print("\nfrom each node: 300 steps toward fresh samples")
for node in top:
    q, t = pl.Q[node].astype(float), pl.T[node]
    outcome, pairs = collections.Counter(), collections.Counter()
    margin_ok = d.module.validate(q.astype(np.float32), d.env, True)
    for _ in range(300):
        idx += 1; q_s, t_s = np.asarray(sam.next(), float), max(vdc(idx), t)
        dist = np.sqrt(np.sum((q_s - q) ** 2) + (W * (t_s - t)) ** 2); k = min(1.0, R / dist)
        q1, t1 = q + (q_s - q) * k, t + (t_s - t) * k
        try:
            qp = np.asarray(d.module.project(np.clip(q1, model.lo, model.hi).astype(np.float32), [d.region_at(setup, t1), setup.com], st), float)
        except ValueError:
            outcome["projection fails"] += 1; continue
        if d.module.validate(qp.astype(np.float32), d.env, True):
            outcome["endpoint valid"] += 1
        else:
            pr = d.pair_names(setup, qp)
            outcome["endpoint invalid: " + ("self-collision" if pr else "joint limit")] += 1
            for p in pr:
                pairs[p] += 1
    print(f"\nnode {node} (t={t:.3f}, valid itself: {margin_ok}): {dict(outcome)}")
    for p, c in pairs.most_common(4):
        print(f"   {c:4d}  {p}")
