"""Tree size against the iteration budget (planner from the problem's start, benchmark settings)."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
print("\nmax_iterations -> tree size (planner from the start, weight 5, range", d.RANGE, ")")
for n in (1, 3, 10, 30, 100, 300, 1000, 3000, 20000):
    d.MAX_ITERATIONS = n
    r = d.plan_with(setup, 5.0)
    print(f"  {n:>6} -> size {list(r.size)}  solved={r.solved}  ms={r.nanoseconds / 1e6:.0f}", flush=True)
