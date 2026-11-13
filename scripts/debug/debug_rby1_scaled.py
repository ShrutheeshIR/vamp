"""Python test of: (1) a metric that weights the base's x and y so their full range counts as much as the largest revolute joint's range,
(2) the base yaw sampled in start_yaw +- pi only. Old goal-walk rule, the C++ steer. Variant C also shrinks the base xy sampling box."""
import collections
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_pyplanner import PyPlanner

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
max_rev = float(np.max((hi - lo)[3:]))                      # the largest revolute range of the non-base joints
yaw0 = float(setup.q_start[2]); x0, y0 = float(setup.q_start[0]), float(setup.q_start[1])
print(f"\nlargest revolute range {max_rev:.2f} rad; start base (x, y, yaw) = ({x0:.2f}, {y0:.2f}, {yaw0:.2f})")


def variant(name, xy_half):
    blo, bhi = lo.copy(), hi.copy()
    blo[2], bhi[2] = yaw0 - np.pi, yaw0 + np.pi              # (2) yaw: start yaw +- pi
    blo[0], bhi[0], blo[1], bhi[1] = x0 - xy_half, x0 + xy_half, y0 - xy_half, y0 + xy_half
    w = np.ones(23); w[0] = w[1] = max_rev / (2 * xy_half)   # (1) full xy range == the largest revolute range
    pl = PyPlanner(setup, d.PARAMETER_WEIGHTS[0], qweights=w, sample_box=(blo, bhi))
    solved = pl.run(3000, (30, 100, 300, 1000, 3000))
    print(f"\n{name}: xy weight {w[0]:.3f}, base box +-{xy_half:g} m, yaw {yaw0 - np.pi:.2f}..{yaw0 + np.pi:.2f}")
    print(f"  solved={solved}; (iteration, tree size, highest t): {[(i, n, round(t, 3)) for i, n, t in pl.history]}")
    print(f"  explore outcomes: {dict(pl.explore_log.most_common(4))}")
    print(f"  walk stops: {dict(pl.walk_log.most_common(3))}", flush=True)


variant("A  as asked: weighted xy + yaw +-pi, xy box +-20 m", 20.0)
variant("C  also xy box +-2 m (not asked, for comparison)", 2.0)
