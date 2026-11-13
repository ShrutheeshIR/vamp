import sys
sys.path.insert(0, "/src/vamp/scripts")
import debug_rby1 as d
problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
print("\nC++ planner, 20001 iterations: project_samples, parameter_weight -> solved, iterations, tree size, ms")
for ps in (False, True):
    d.PROJECT_SAMPLES = ps
    for w in (5.0, 1.0, 0.1):
        r = d.plan_with(setup, w)
        print(f"  project_samples={ps!s:<5} weight={w:<4g} -> solved={r.solved!s:<5} iterations={int(r.iterations):>6} tree={list(r.size)} ms={r.nanoseconds / 1e6:.0f}", flush=True)
