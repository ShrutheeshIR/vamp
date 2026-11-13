"""For the bucket-2 failing plans (no valid IK found at some t): is it (A) the two hands' SE3 poses themselves collide, or (B) the poses are fine and no collision-free configuration was found?
Per bad t: run IK restarts and keep every CONVERGED solution (collision-free or not). The hand-side boxes (last arm link, FT sensor, gripper body, two fingers, wrist camera, 6 per hand)
are rigid with the end-effector pose, so a collision between a left-hand and a right-hand box is the same in every configuration that reaches that pose:
  pose collision   : a converged solution has a left-hand/right-hand box pair in collision  -> the SE3 poses themselves collide (VAMP's hand boxes)
  config collision : converged solutions exist, none with a hand/hand pair                   -> poses fine, those configurations collide elsewhere (arm/torso)
  no convergence   : no restart converged                                                    -> the pose is kinematically unreachable (bounds, reach, CoM)
  valid found      : a collision-free solution was found now (the earlier scan missed it)
Window verdict: pose collision if any bad t is one; else config collision if any; else no convergence.
Usage: debug_rby1_poseclass.py SHARD N_SHARDS [sid ...]"""
import collections, glob, json, os, sys, time, traceback
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from debug_rby1_cases import build_case_at, _load, STRIDE
from diffik_benchmark.methods import make_method
from diffik_benchmark.runner import cut_time
from diffik_benchmark.joint_trajectory import MethodExit
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams

NW, MW, TRIES = 16, 4, 40
shard, n_shards = int(sys.argv[1]), int(sys.argv[2]); only = [int(a) for a in sys.argv[3:]]
spec = get_robot("rby1"); backend = DrakeBackend(spec); sp = ScorerParams(); tool_ref = spec.tool_ref_poses()
method = make_method("vamp_curve_rrtc", dict(range="1.0", parameter_weight="23", walk_by_parameter="true", base_metric_weight="true", recenter_yaw="true", max_knot_step="0.01", segment="false"))
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
rng = np.random.default_rng(100 + shard)
LEFT, RIGHT = set(range(64, 70)), set(range(70, 76))   # indices of the hand boxes in the collision-geometry list (44 spheres, then the 32 boxes: 20..25 left hand, 26..31 right hand)


def tag(setup, i): return setup.model.names[i]


def converged_solutions(setup, t, n):
    ee = np.array(d.pose_at(setup, t))[:, 4:6].mean(axis=0); sols = []
    seeds = [setup.q_start.astype(float)]
    for k in range(n):
        if k < len(seeds): sd = seeds[k]
        else:
            sd = rng.uniform(lo, hi); sd[:2] = ee + rng.uniform(-1.2, 1.2, 2); sd[2] = rng.uniform(-np.pi, np.pi)
        o = d.walk_project(setup, sd, t, "BoundedLS")
        if o is not None: sols.append(o)
    return sols


def classify_t(setup, t):
    sols = converged_solutions(setup, t, TRIES)
    if not sols: return "no convergence", {}, 0
    pc = collections.Counter(); kind = "config collision"
    for o in sols:
        if d.module.validate(o.astype(np.float32), d.env, True): return "valid found", {}, len(sols)
        pairs = d.module.debug(o.astype(np.float32), d.env)[1]
        hh = [(a, b) for a, b in pairs if (a in LEFT and b in RIGHT) or (b in LEFT and a in RIGHT)]
        if hh:
            kind = "pose collision"
            for a, b in hh: pc[" <-> ".join(sorted((tag(setup, a), tag(setup, b))))] += 1
        else:
            for a, b in pairs: pc["(config) " + " <-> ".join(sorted((tag(setup, a), tag(setup, b))))] += 1
    return kind, dict(pc.most_common(3)), len(sols)


rows = [json.loads(l) for f in sorted(glob.glob("/src/diffik-benchmark/scratch/endclass/*.jsonl")) for l in open(f)]
todo = sorted([(r["ds"], r["sid"], r["plan"], r["bad_ts"]) for r in rows if r["bucket"].startswith("2")])
todo = [x for x in todo if x[1] in only] if only else [x for i, x in enumerate(todo) if i % n_shards == shard]
os.makedirs("/src/diffik-benchmark/scratch/poseclass", exist_ok=True)
out = open(f"/src/diffik-benchmark/scratch/poseclass/{shard}.jsonl", "w")
for ds, sid, k, bad in todo:
    t0 = time.time(); st, eps = _load(ds); ep = eps[int(st.episode_index[sid])]
    q = np.asarray(st.q[sid], float).copy(); s = int(st.start_index[sid]); X = st.X_W_ref[sid]
    try:
        for i in range(k + 1):
            seg = ep.segment(s, NW, STRIDE).transformed(X, tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)
            plan = method(backend, q, seg)
            if i == k: break
            outp = plan.executed_prefix(cut_time(plan, MW, NW)); q = np.asarray(outp.trajectory.knots[-1], float).copy(); s += STRIDE * MW
        setup = build_case_at(ds, sid, q, s, NW)
        per = []
        for t in bad:
            kind, pairs, n = classify_t(setup, float(t)); per.append(dict(t=t, kind=kind, pairs=pairs, n_converged=n))
        kinds = collections.Counter(p["kind"] for p in per)
        verdict = ("pose collision" if kinds["pose collision"] else "config collision" if kinds["config collision"] else "valid found" if kinds["valid found"] else "no convergence")
        rec = dict(ds=ds, sid=sid, plan=k, verdict=verdict, kinds=dict(kinds), per_t=per, replay_still_fails=(plan.exit == MethodExit.NO_SOLUTION))
    except Exception:
        rec = dict(ds=ds, sid=sid, plan=k, verdict="raised", error=traceback.format_exc().splitlines()[-1])
    rec["seconds"] = round(time.time() - t0, 1); out.write(json.dumps(rec) + "\n"); out.flush()
