"""For the chains (N = 16, M = 4, new settings) that stopped with NOT_FINISHED: replay to the failing plan and classify it.
 start invalid       : VAMP's model rejects the plan's start configuration
 start unprojectable : the start cannot be projected onto the first waypoint
 walk stalls         : the exact walk (small steps, 60 seeds) cannot track the 17 poses in VAMP's model (a model limit)
 planner failure     : the exact walk reaches the end, the C++ planner (range 1.0, weight 23, three changes) does not
 solved here         : the C++ planner solves it in this frame (the method failed for another reason)"""
import collections, contextlib, csv, glob, io, json, os
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from dataclasses import replace
from diffik_benchmark import STARTS_DIR, TRAJECTORIES_DIR, transforms as tf
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.methods import make_method
from diffik_benchmark.robots import get_robot
from diffik_benchmark.runner import run_chain
from diffik_benchmark.scoring import ScorerParams
from diffik_benchmark.starts import StartSet, starts_path
from diffik_benchmark.trajectories import iter_episodes

N, M, STRIDE = 16, 4, 6
spec = get_robot("rby1"); b = DrakeBackend(spec); sp = ScorerParams(); tool_ref = spec.tool_ref_poses()
method = make_method("vamp_curve_rrtc", {"segment": "false", "range": "1.0", "parameter_weight": "23", "walk_by_parameter": "true", "base_metric_weight": "true", "recenter_yaw": "true"})
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
with contextlib.redirect_stdout(io.StringIO()):
    model = d.load_model()
names = list(d.module.joint_names())
cases = []
for f in sorted(glob.glob("/src/diffik-benchmark/scratch/big/chain32_*_new.jsonl")):
    ds = os.path.basename(f)[len("chain32_"):-len("_new.jsonl")]
    for line in open(f):
        r = json.loads(line)
        if "done" not in r and r["why"] == "NOT_FINISHED":
            cases.append((ds, r["sid"], r["plans"]))
print(f"{len(cases)} chains stopped with NOT_FINISHED", flush=True)
cls = collections.Counter(); rows = []
cache = {}
for ds, sid, n_plans in cases:
    if ds not in cache:
        st = StartSet.load(starts_path("rby1", ds, STARTS_DIR))
        eps = {int(e.index): e for e in iter_episodes(TRAJECTORIES_DIR / ds, include_flagged=True)}
        cache[ds] = (st, eps)
    st, eps = cache[ds]; ep = eps[int(st.episode_index[sid])]
    scores, outs, segs, plans = run_chain(b, ep, st.q[sid], st.X_W_ref[sid], int(st.start_index[sid]), N, STRIDE, method, sp, M=M)
    k = len(scores) - 1                                              # the failing plan
    q_bench = np.asarray(st.q[sid], float) if k == 0 else np.asarray(scores[k - 1].final_q, float)
    s = int(st.start_index[sid]) + STRIDE * M * k
    seg = ep.segment(s, N, STRIDE).transformed(st.X_W_ref[sid], tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)
    # the plan's frame: base xy at the origin, yaw 0 (what the method does)
    to_vamp = np.array([[n.replace("/", "_") for n in b.scoped_joint_names].index(n.replace("/", "_")) for n in names])
    q0 = q_bench[to_vamp].copy(); shift = np.zeros(23); shift[:2] = q0[:2]; q0 -= shift
    yaw0 = float(q0[2]); R = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]]); qz = np.array([np.cos(-yaw0 / 2), 0, 0, np.sin(-yaw0 / 2)])
    q0[2] = 0.0
    pos = seg.pos[:, d.EE_ORDER] - np.array([shift[0], shift[1], 0.0]); pos = pos.copy(); pos[..., :2] = pos[..., :2] @ R
    quat = tf.quat_mul(qz, seg.quat[:, d.EE_ORDER])
    problem = d.Problem(sid, N, STRIDE, int(st.episode_index[sid]), s, st.X_W_ref[sid], q0, q_bench, shift, to_vamp, pos, quat)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            setup = d.make_setup(problem, model)
    except Exception as e:
        cls["start unprojectable"] += 1; rows.append((ds, sid, k, "start unprojectable")); continue
    if not d.module.validate(setup.q_start.astype(np.float32), d.env, True):
        cls["start invalid"] += 1; rows.append((ds, sid, k, "start invalid")); continue
    path, rej = d.walk_reference(setup, "BoundedLS")
    r = d.plan_with(setup, 23.0, range_=1.0)
    if rej:
        c = "walk stalls" + (" (but the planner solves it)" if r.solved else "")
    else:
        c = "solved here" if r.solved else "planner failure"
    cls[c] += 1; rows.append((ds, sid, k, c, round(path[-1][0], 2)))
    print(f"{ds[:12]} sid {sid:>4} plan {k:>3}: {c}", flush=True)
print("\nfailing plan of each NOT_FINISHED chain:", dict(cls))
print("planner failures:", [(ds[:9], sid, k) for ds, sid, k, c, *_ in rows if c == "planner failure"])
json.dump(rows, open("/src/diffik-benchmark/scratch/chainfail.json", "w"))
