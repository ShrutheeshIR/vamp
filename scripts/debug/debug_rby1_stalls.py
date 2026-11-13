"""The valid-start starts the planner does not solve and the exact walk cannot track (edge check 0.01 j): where it stalls and what rejects it."""
import collections, glob, json, os, sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_cases import build_case

big = "/src/diffik-benchmark/scratch/big/"
qp = {}; cases = []
for f in glob.glob(big + "all32_*_qp.jsonl"):
    ds = os.path.basename(f)[len("all32_"):-len("_qp.jsonl")]
    for l in open(f):
        r = json.loads(l)
        if "done" not in r: qp[(ds, r["sid"])] = r["solved"]
for f in glob.glob(big + "all32_*_new.jsonl"):
    ds = os.path.basename(f)[len("all32_"):-len("_new.jsonl")]
    for l in open(f):
        r = json.loads(l)
        if "done" not in r and r["valid"] and not r["solved"]: cases.append((ds, r["sid"]))
rows = []
for ds, sid in cases:
    setup = build_case(ds, sid)
    if setup is None or not d.module.validate(setup.q_start.astype(np.float32), d.env, True): continue
    if d.plan_with(setup, 23.0, range_=1.0).solved: continue
    path, rej = d.walk_reference(setup, "BoundedLS")
    if not rej: rows.append(dict(ds=ds, sid=sid, qp=qp[(ds, sid)], t=1.0, why="walk reaches the end", pairs={})); continue
    why = collections.Counter(w for w, _ in rej)
    pairs = collections.Counter(); 
    for w, o in rej:
        if w == "self-collision" and o is not None: pairs.update(set(d.pair_names(setup, o)))
    rows.append(dict(ds=ds, sid=sid, qp=qp[(ds, sid)], t=path[-1][0], why=dict(why), pairs=dict(pairs)))
    print(f"{ds[:12]} sid {sid:>4} QP {'solves' if qp[(ds, sid)] else 'fails '} stalls at t={path[-1][0]:.2f}: {dict(why)} | pairs {dict(pairs.most_common(2))}", flush=True)
json.dump(rows, open("/src/diffik-benchmark/scratch/stalls.json", "w"))
print("\n==== summary")
for label, sel in (("QP solves", [r for r in rows if r["qp"]]), ("QP fails", [r for r in rows if not r["qp"]])):
    print(f"\n{label}: {len(sel)} starts")
    print("  stall t:", dict(sorted(collections.Counter(round(r["t"] * 10) / 10 for r in sel).items())))
    dom = collections.Counter(max(r["why"], key=r["why"].get) if isinstance(r["why"], dict) and r["why"] else str(r["why"]) for r in sel)
    print("  dominant rejection at the stall:", dict(dom))
    pc = collections.Counter(); [pc.update(r["pairs"].keys()) for r in sel]
    print("  colliding pairs (starts):", dict(pc.most_common(6)))
