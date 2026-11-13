"""Step 2 for the starts that fail with all three changes: does the exact walk reach t = 1 (a path exists in VAMP's model)? If it stalls, what rejects
the candidates there? Also whether the C++ planner with the changes (in this script's frame: yaw not recentred) fails here too."""
import collections, contextlib, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
print("sid   start yaw  C++ planner(it)   walk BoundedLS               walk InnerLM")
for sid in [int(a) for a in sys.argv[1:]]:
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
        r = d.plan_with(setup, 5.0)
        out = []
        for method in ("BoundedLS", "InnerLM"):
            path, rej = d.walk_reference(setup, method)
            out.append(f"t={path[-1][0]:.2f} " + (str(dict(collections.Counter(w for w, _ in rej).most_common(2))) if rej else "end"))
        print(f"{sid:>3}   {float(problem.q0[2]):+.2f}      {r.solved!s:<5}({int(r.iterations):>5})   {out[0]:<28} {out[1]}", flush=True)
    except Exception as e:
        print(f"{sid:>3}   error {type(e).__name__}: {str(e)[:70]}", flush=True)
