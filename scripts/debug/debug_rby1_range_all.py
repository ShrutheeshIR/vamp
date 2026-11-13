"""Planner only (C++, goal walk accepts t, base xy weight, parameter weight 23): range 0.25 / 1.0 / 1.5 over all valid starts of the first 100 certified."""
import contextlib, csv, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

R = (0.25, 1.0, 1.5)
cert = [int(r["start_id"]) for r in csv.DictReader(open("/src/diffik-benchmark/results/instance_sets/rby1__nymeria_L60x180_v01__N16M4.csv")) if r["label"] == "certified"][:100]
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
res = {r: dict(solved=0, ms=[], ms_solved=[], fail=[], it=[]) for r in R}; n = 0
for sid in cert:
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
        if not d.module.validate(setup.q_start.astype(np.float32), d.env, True): continue
    except Exception:
        continue
    n += 1
    for rg in R:
        r = d.plan_with(setup, 23.0, range_=rg); ms = r.nanoseconds / 1e6
        res[rg]["ms"].append(ms)
        if r.solved: res[rg]["solved"] += 1; res[rg]["ms_solved"].append(ms); res[rg]["it"].append(int(r.iterations))
        else: res[rg]["fail"].append(sid)
print(f"{n} starts (valid start in VAMP's model), N = 31, parameter weight 23")
print(f"{'range':>6} {'solved':>8} {'mean ms (all)':>14} {'median ms (solved)':>19} {'mean ms (solved)':>17} {'median iterations (solved)':>27}   unsolved")
for rg in R:
    r = res[rg]
    print(f"{rg:>6g} {r['solved']:>5}/{n:<2} {np.mean(r['ms']):>14.0f} {np.median(r['ms_solved']):>19.0f} {np.mean(r['ms_solved']):>17.0f} {np.median(r['it']):>27.0f}   {r['fail']}")
