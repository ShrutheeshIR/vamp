"""Two predictions of the goal-step explanation (see phase 0): planner from the walk's state at t0 with a parameter weight small
enough that one goal step can move t past the hump. Predicted: solves."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
path, _ = d.walk_reference(setup, "BoundedLS")
by_t = {round(t, 2): q for t, q in path}
print(f"\nrange={d.RANGE}; the goal step moves t by at most range / weight")
print("   t0    weight  max dt/step  solved  iterations")
for t0, w in ((0.7, 5.0), (0.7, 1.0), (0.7, 0.5), (0.2, 1.0), (0.2, 0.3), (0.0, 1.0), (0.0, 0.3), (0.0, 0.1)):
    r = d.plan_with(setup, w, q_start=by_t[t0], start_t=t0)
    print(f"  {t0:.2f}  {w:6g}  {d.RANGE / w:10.2f}   {r.solved!s:<6}  {int(r.iterations):>9}", flush=True)
