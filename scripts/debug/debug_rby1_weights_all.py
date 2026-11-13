"""Planner only (C++, goal walk accepts t, base xy weight): per start, which parameter weights solve it, next to the reference's joint speed (median
|dq| per unit t along the exact walk). Prints the starts whose result changes with the weight, and the totals."""
import contextlib, csv, io
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

W = (5.0, 10.0, 17.0, 25.0, 50.0, 100.0)
cert = [int(r["start_id"]) for r in csv.DictReader(open("/src/diffik-benchmark/results/instance_sets/rby1__nymeria_L60x180_v01__N16M4.csv")) if r["label"] == "certified"][:100]
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
tot = np.zeros(len(W), int); n = 0; rows = []
for sid in cert:
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            problem = d.load_problem(sid, 31, 6); model = d.load_model(); setup = d.make_setup(problem, model)
        if not d.module.validate(setup.q_start.astype(np.float32), d.env, True): continue
    except Exception:
        continue
    n += 1
    res = [bool(d.plan_with(setup, w).solved) for w in W]
    tot += res
    if len(set(res)) > 1 or not res[-1]:
        path, rej = d.walk_reference(setup, "BoundedLS")
        speed = np.median(np.linalg.norm(np.diff([q for _, q in path], axis=0), axis=1)) / 0.02
        rows.append((sid, res, speed, path[-1][0]))
    print(f"\rstart {sid}", end="", flush=True)
print(f"\n{n} starts (valid start in VAMP's model); planner solved per weight: " + ", ".join(f"w={w:g}: {t}" for w, t in zip(W, tot)))
print("\nstarts whose result changes with the weight, or unsolved at the largest:")
print("sid   " + "  ".join(f"w={w:g}".rjust(6) for w in W) + "   joint speed (rad per unit t)   exact walk reaches t")
for sid, res, speed, tend in rows:
    print(f"{sid:>3}   " + "  ".join(("yes" if r else "no").rjust(6) for r in res) + f"   {speed:>10.1f}   {tend:>10.2f}")
