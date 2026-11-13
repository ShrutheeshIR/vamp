import sys
sys.path.insert(0, "/src/vamp/scripts")
import debug_rby1 as d
problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
print("\nbounds yaw:", float(d.module.lower_bounds()[2]), float(d.module.upper_bounds()[2]))
print("C++ planner, sid 26 (start yaw", round(float(setup.q_start[2]), 2), "), weight 5: variant -> solved, iterations, tree, ms")
for name, wp, bw in (("yaw bounds +-pi only", False, False), ("+ goal walk accepts t", True, False), ("+ base xy metric weight", False, True), ("all three", True, True)):
    d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = wp, bw
    r = d.plan_with(setup, 5.0)
    print(f"  {name:<26} solved={r.solved!s:<5} iterations={int(r.iterations):>6} tree={list(r.size)} ms={r.nanoseconds / 1e6:.0f}", flush=True)
