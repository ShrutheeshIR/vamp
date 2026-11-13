"""Which of the two changes matters? Old goal-walk rule, 5000 iterations. Checks the solved path with interpolate_curve_path."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:])); yaw0 = float(setup.q_start[2])


def run(name, yaw_box, xy_weight):
    blo, bhi = lo.copy(), hi.copy()
    if yaw_box:
        blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi
    w = np.ones(23); w[0] = w[1] = xy_weight
    pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0], qweights=w, sample_box=(blo, bhi))
    solved = pl.run(5000)
    msg = f"{name:<44} solved={solved!s:<5} iterations={pl.iterations:>5} tree={len(pl.Q):>5}"
    if solved:
        i, idx = pl.solution, []
        while True:
            idx.append(i)
            if i == 0: break
            i = pl.parent[i]
        Q = [pl.Q[j] for j in idx[::-1]]; T = [pl.T[j] for j in idx[::-1]]
        ok = d.module.interpolate_curve_path([q.astype(np.float32) for q in Q], [float(t) for t in T], d.env, pl.cs, pl.settings, d.MAX_KNOT_STEP).interpolated
        msg += f" path: {len(Q)} nodes, valid+on curve (interpolate_curve_path): {ok}"
    print(msg, flush=True)


run("neither (old behaviour)", False, 1.0)
run("yaw +-pi only", True, 1.0)
run("xy weight only (6.0/40 = 0.15)", False, max_rev / 40)
run("both", True, max_rev / 40)
