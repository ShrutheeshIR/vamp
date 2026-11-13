"""Tree size at failure for a few planner runs (what the result exposes)."""
import sys
sys.path.insert(0, "/src/vamp/scripts")
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
path, _ = d.walk_reference(setup, "BoundedLS")
by_t = {round(t, 2): q for t, q in path}
r = d.plan_with(setup, 5.0)
print("\nresult attributes:", [a for a in dir(r) if not a.startswith("_")])
for t0, w in ((0.0, 5.0), (0.7, 0.5), (0.8, 5.0), (0.7, 5.0)):
    r = d.plan_with(setup, w, q_start=by_t[t0], start_t=t0)
    print(f"t0={t0} weight={w:g}: solved={r.solved} iterations={int(r.iterations)} ms={r.nanoseconds / 1e6:.0f} tree size={list(r.size)}", flush=True)
