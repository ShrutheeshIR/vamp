"""Planner only (C++, goal walk accepts t, base xy weight), all valid starts of the first 100 certified: parameter weight 5 against #dof (23)."""
import contextlib, csv, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

W = (5.0, 23.0)
cert = [int(r["start_id"]) for r in csv.DictReader(open("/src/diffik-benchmark/results/instance_sets/rby1__nymeria_L60x180_v01__N16M4.csv")) if r["label"] == "certified"][:100]
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
res = {w: dict(solved=0, ms=[], ms_solved=[], fail=[]) for w in W}; n = 0
for sid in cert:
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
        if not d.module.validate(setup.q_start.astype(np.float32), d.env, True): continue
    except Exception:
        continue
    n += 1
    for w in W:
        r = d.plan_with(setup, w); ms = r.nanoseconds / 1e6
        res[w]["ms"].append(ms)
        if r.solved: res[w]["solved"] += 1; res[w]["ms_solved"].append(ms)
        else: res[w]["fail"].append(sid)
print(f"{n} starts (valid start in VAMP's model), N = 31")
print(f"{'weight':>8} {'solved':>8} {'mean ms (all)':>14} {'median ms (solved)':>19} {'mean ms (solved)':>17}   unsolved")
for w in W:
    r = res[w]
    print(f"{w:>8g} {r['solved']:>5}/{n:<2} {np.mean(r['ms']):>14.0f} {np.median(r['ms_solved']):>19.0f} {np.mean(r['ms_solved']):>17.0f}   {r['fail']}")
