"""The 112 starts (valid in VAMP's model) where the new planner finds no solution for the first 32-pose window: classify each, next to whether the QP solved it.
 walk stalls     : the exact walk (small steps, 60 seeds) cannot track the window in VAMP's model
 planner failure : the exact walk reaches the end, the C++ planner (range 1.0, weight 23, three changes) does not
 planner solves  : the C++ planner solves it in this frame (the method failed for another reason)"""
import collections, contextlib, glob, io, json, os
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from diffik_benchmark import STARTS_DIR, TRAJECTORIES_DIR, transforms as tf
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams
from diffik_benchmark.starts import StartSet, starts_path
from diffik_benchmark.trajectories import iter_episodes

N, STRIDE = 31, 6
spec = get_robot("rby1"); b = DrakeBackend(spec); sp = ScorerParams(); tool_ref = spec.tool_ref_poses()
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
d.MAX_KNOT_STEP = 0.01   # edge check of the exact walk, as the benchmark (max_knot_step); 0.1 let the walk tunnel through thin collisions
with contextlib.redirect_stdout(io.StringIO()):
    model = d.load_model()
names = list(d.module.joint_names())
to_vamp = np.array([[n.replace("/", "_") for n in b.scoped_joint_names].index(n.replace("/", "_")) for n in names])
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
print(f"{len(cases)} starts without a solution (valid start)", flush=True)
cache = {}; res = collections.Counter(); planner_fail = []
for ds, sid in cases:
    if ds not in cache:
        cache[ds] = (StartSet.load(starts_path("rby1", ds, STARTS_DIR)), {int(e.index): e for e in iter_episodes(TRAJECTORIES_DIR / ds, include_flagged=True)})
    st, eps = cache[ds]; ep = eps[int(st.episode_index[sid])]
    s = int(st.start_index[sid]); q_bench = np.asarray(st.q[sid], float)
    seg = ep.segment(s, N, STRIDE).transformed(st.X_W_ref[sid], tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)
    q0 = q_bench[to_vamp].copy(); shift = np.zeros(23); shift[:2] = q0[:2]; q0 -= shift
    yaw0 = float(q0[2]); R = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]]); qz = np.array([np.cos(-yaw0 / 2), 0, 0, np.sin(-yaw0 / 2)])
    q0[2] = 0.0
    pos = seg.pos[:, d.EE_ORDER] - np.array([shift[0], shift[1], 0.0]); pos = pos.copy(); pos[..., :2] = pos[..., :2] @ R
    quat = tf.quat_mul(qz, seg.quat[:, d.EE_ORDER])
    problem = d.Problem(sid, N, STRIDE, int(st.episode_index[sid]), s, st.X_W_ref[sid], q0, q_bench, shift, to_vamp, pos, quat)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            setup = d.make_setup(problem, model)
        valid = d.module.validate(setup.q_start.astype(np.float32), d.env, True)
    except Exception:
        res[("start unprojectable/invalid in this frame", qp[(ds, sid)])] += 1; continue
    if not valid:
        res[("start unprojectable/invalid in this frame", qp[(ds, sid)])] += 1; continue
    path, rej = d.walk_reference(setup, "BoundedLS")
    r = d.plan_with(setup, 23.0, range_=1.0)
    c = "planner solves (frame differs)" if r.solved else ("walk stalls" if rej else "planner failure")
    res[(c, qp[(ds, sid)])] += 1
    if c == "planner failure": planner_fail.append((ds, sid, bool(qp[(ds, sid)])))
    print(f"{ds[:12]} sid {sid:>4}: {c:<30} QP solved: {qp[(ds, sid)]}", flush=True)
print("\nclass x QP:")
for (c, q), n in sorted(res.items()): print(f"  {c:<42} QP {'solves' if q else 'fails ':<6} {n}")
print("planner failures:", planner_fail)
json.dump(planner_fail, open("/src/diffik-benchmark/scratch/planner_fail_list_floor.json", "w"))
