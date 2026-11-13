"""Classify the failing plan of a chain (N16 M4, planner only, scorer ignored) in VAMP's model. Per start (argv: shard n_shards): replay the chain with the method until its
first plan with no solution, then on that plan's window (16 poses, stride 6) from the chain's configuration:
  reproduce : the C++ planner in this frame (range 1.0, weight 23, three flags, trace-gap floor) also fails
  poses     : random-restart IK (continuation seeds + random restarts) at every waypoint's t and on a 51-point t grid: collision-free, in bounds, CoM ok
  alt starts: up to 10 diverse valid IK solutions at t=0 -> the C++ planner (3000 iterations) from each
  walk      : exact walk (edge 0.01 j) from the chain's configuration
Bucket (first that applies): 1 a different start solves; 2(i) end pose has no valid IK; 2(ii) start pose has none; 2(iii) start and end have, an internal t has none;
3a the exact walk reaches the end, the planner does not; 3b the walk stalls (pairs recorded)."""
import collections, contextlib, io, json, os, sys, time, traceback
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_cases import build_case_at, _load, N as _N, STRIDE
from diffik_benchmark.methods import make_method
from diffik_benchmark.runner import cut_time
from diffik_benchmark.joint_trajectory import MethodExit
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams

NW, MW = 16, 4
R_RESTARTS, T_GRID, ALT_MAX, ALT_ITERS = 300, 51, 10, 3000
shard, n_shards = int(sys.argv[1]), int(sys.argv[2]); only = [int(a) for a in sys.argv[3:]]
spec = get_robot("rby1"); backend = DrakeBackend(spec); sp = ScorerParams(); tool_ref = spec.tool_ref_poses()
method = make_method("vamp_curve_rrtc", dict(range="1.0", parameter_weight="23", walk_by_parameter="true", base_metric_weight="true", recenter_yaw="true", max_knot_step="0.01", segment="false"))
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
rng = np.random.default_rng(shard)


def valid(q):
    return d.module.validate(np.asarray(q, np.float32), d.env, True)


def find_ik(setup, t, seeds, n_random):
    """A collision-free, in-bounds solution at pose(t): first the given seeds (and noisy copies), then random restarts. Returns (q or None, tries)."""
    ee = np.array(d.pose_at(setup, t))[:, 4:6].mean(axis=0)
    tries = 0
    cand = []
    for sd in seeds:
        cand.append(sd)
        for sig in (0.05, 0.15, 0.4): cand.append(sd + sig * rng.standard_normal(len(sd)))
    for c in cand:
        tries += 1; o = d.walk_project(setup, c, t, "BoundedLS")
        if o is not None and valid(o): return o, tries
    for _ in range(n_random):
        tries += 1; sd = rng.uniform(lo, hi); sd[:2] = ee + rng.uniform(-1.2, 1.2, 2); sd[2] = rng.uniform(-np.pi, np.pi)
        o = d.walk_project(setup, sd, t, "BoundedLS")
        if o is not None and valid(o): return o, tries
    return None, tries


def scan_poses(setup):
    ts = sorted(set(float(x) for x in setup.taus) | set(np.linspace(0, 1, T_GRID)))
    found, bad, prev = {}, [], [setup.q_start.astype(float)]
    for t in ts:
        o, _ = find_ik(setup, t, prev[-2:], R_RESTARTS)
        if o is None: bad.append(round(t, 4))
        else: found[t] = o; prev.append(o)
    return ts, found, bad


def alt_starts(setup, q_start):
    sols = []
    for _ in range(40):
        o, _ = find_ik(setup, 0.0, [], 25)
        if o is not None and np.linalg.norm(o - q_start) > 0.05: sols.append(o)
        if len(sols) >= 30: break
    if not sols: return []
    pick = [sols[0]]
    while len(pick) < min(ALT_MAX, len(sols)):
        dist = np.min([np.linalg.norm(np.array(sols) - p, axis=1) for p in pick], axis=0); pick.append(sols[int(np.argmax(dist))])
    return pick


def classify(ds, sid, k, q_bench, s):
    setup = build_case_at(ds, sid, q_bench, s, NW)
    rec = dict(ds=ds, sid=sid, plan=k)
    if setup is None or not valid(setup.q_start): rec.update(bucket="start unprojectable/invalid in this frame"); return rec
    d.MAX_ITERATIONS = 20000
    r = d.plan_with(setup, 23.0, range_=1.0); rec["reproduced_failure"] = not r.solved
    if r.solved: rec.update(bucket="planner solves in this frame (frame differs)"); return rec
    ts, found, bad = scan_poses(setup)
    end_bad, start_bad = 1.0 in [round(x, 4) for x in bad], 0.0 in [round(x, 4) for x in bad]
    rec.update(n_poses=len(ts), bad_ts=bad, end_bad=end_bad, start_bad=start_bad)
    alts = alt_starts(setup, setup.q_start.astype(float)); d.MAX_ITERATIONS = ALT_ITERS
    solved = [bool(d.plan_with(setup, 23.0, range_=1.0, q_start=a.astype(np.float32), start_t=0.0).solved) for a in alts]
    d.MAX_ITERATIONS = 20000
    rec.update(n_alt=len(alts), n_alt_solved=int(sum(solved)))
    path, rej = d.walk_reference(setup, "BoundedLS"); rec.update(walk_t=round(path[-1][0], 3), walk_reached=not rej)
    if rej:
        rec["walk_reasons"] = dict(collections.Counter(w for w, _ in rej)); pc = collections.Counter()
        for w, o in rej:
            if w == "self-collision" and o is not None: pc.update(set(d.pair_names(setup, o)))
        rec["walk_pairs"] = dict(pc.most_common(3))
    if any(solved): rec["bucket"] = "1 a different start solves"
    elif end_bad: rec["bucket"] = "2(i) end pose has no valid IK"
    elif start_bad: rec["bucket"] = "2(ii) start pose has no valid IK"
    elif bad: rec["bucket"] = "2(iii) start and end have IK, an internal t has none"
    elif not rej: rec["bucket"] = "3a exact walk reaches the end, planner does not"
    else: rec["bucket"] = "3b every pose has IK, exact walk stalls, no alt start solves"
    return rec


os.makedirs("/src/diffik-benchmark/scratch/endclass", exist_ok=True)
fails = []
for f in sorted(__import__("glob").glob("/src/diffik-benchmark/scratch/noscore/*.jsonl")):
    ds = os.path.basename(f).rsplit("_", 1)[0]
    for l in open(f):
        r = json.loads(l)
        if r["stop"] == "no solution": fails.append((ds, r["sid"], r["plans"]))
fails = [x for x in fails if (not only and hash((x[0], x[1])) % 1 == 0)] if not only else [x for x in fails if x[1] in only]
fails = [x for i, x in enumerate(sorted(fails)) if (not only and i % n_shards == shard) or only]
out = open(f"/src/diffik-benchmark/scratch/endclass/{shard}.jsonl", "w")
for ds, sid, k in fails:
    t0 = time.time(); st, eps = _load(ds); ep = eps[int(st.episode_index[sid])]
    q = np.asarray(st.q[sid], float).copy(); s = int(st.start_index[sid]); X = st.X_W_ref[sid]; stuck = None
    try:
        for i in range(k + 1):
            seg = ep.segment(s, NW, STRIDE).transformed(X, tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)
            plan = method(backend, q, seg)
            if plan.exit == MethodExit.NO_SOLUTION: stuck = i; break
            outp = plan.executed_prefix(cut_time(plan, MW, NW)); q = np.asarray(outp.trajectory.knots[-1], float).copy(); s += STRIDE * MW
        rec = dict(ds=ds, sid=sid, plan=k, replay_matches=(stuck == k), start_valid_in_method=plan.extra.get("start_valid"))
        if stuck is None: rec["bucket"] = "replay did not fail"
        else:
            rec.update(classify(ds, sid, stuck, q, s)); rec["replay_matches"] = (stuck == k)
    except Exception:
        rec = dict(ds=ds, sid=sid, plan=k, bucket="raised", error=traceback.format_exc().splitlines()[-1])
    rec["seconds"] = round(time.time() - t0, 1); out.write(json.dumps(rec) + "\n"); out.flush()
