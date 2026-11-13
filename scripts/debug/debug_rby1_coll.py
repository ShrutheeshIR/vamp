"""View the collision between two consecutive points of the exact walk (BoundedLS, edge check 0.1 j), lane by lane.
Usage: python3 /src/vamp/scripts/debug_rby1_coll.py DATASET SID [T=0.72] [--check]
Sequences in the viewer (slider "frame"):
  "Walk edge"        : walk point at T, then the 8 lanes of trace() (the line between the two points, each lane projected onto the reference pose at its t), then the next walk point
  "Unprojected line" : the same 8 lanes before projection (straight line in joints, t interpolated)
Colliding primitives are red; the text box lists the colliding pairs."""
import contextlib, io, sys, time
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
from diffik_benchmark import STARTS_DIR, TRAJECTORIES_DIR, transforms as tf
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams
from diffik_benchmark.starts import StartSet, starts_path
from diffik_benchmark.trajectories import iter_episodes

args = [a for a in sys.argv[1:] if not a.startswith("--")]
ds, sid, T_AT = args[0], int(args[1]), float(args[2]) if len(args) > 2 else 0.72
N, STRIDE = 31, 6
spec = get_robot("rby1"); b = DrakeBackend(spec); sp = ScorerParams(); tool_ref = spec.tool_ref_poses()
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
with contextlib.redirect_stdout(io.StringIO()):
    model = d.load_model()
names = list(d.module.joint_names())
to_vamp = np.array([[n.replace("/", "_") for n in b.scoped_joint_names].index(n.replace("/", "_")) for n in names])
st = StartSet.load(starts_path("rby1", ds, STARTS_DIR)); eps = {int(e.index): e for e in iter_episodes(TRAJECTORIES_DIR / ds, include_flagged=True)}
ep = eps[int(st.episode_index[sid])]; s = int(st.start_index[sid]); q_bench = np.asarray(st.q[sid], float)
seg = ep.segment(s, N, STRIDE).transformed(st.X_W_ref[sid], tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)
q0 = q_bench[to_vamp].copy(); shift = np.zeros(23); shift[:2] = q0[:2]; q0 -= shift
yaw0 = float(q0[2]); R = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]]); qz = np.array([np.cos(-yaw0 / 2), 0, 0, np.sin(-yaw0 / 2)])
q0[2] = 0.0
pos = seg.pos[:, d.EE_ORDER] - np.array([shift[0], shift[1], 0.0]); pos = pos.copy(); pos[..., :2] = pos[..., :2] @ R
quat = tf.quat_mul(qz, seg.quat[:, d.EE_ORDER])
problem = d.Problem(sid, N, STRIDE, int(st.episode_index[sid]), s, st.X_W_ref[sid], q0, q_bench, shift, to_vamp, pos, quat)
with contextlib.redirect_stdout(io.StringIO()):
    setup = d.make_setup(problem, model)
d.MAX_KNOT_STEP = 0.1
path, rej = d.walk_reference(setup, "BoundedLS")
wt = np.array([p[0] for p in path]); wq = np.array([p[1] for p in path])
ia = int(np.argmin(np.abs(wt - T_AT))); ib = min(ia + 1, len(wt) - 1)
qa, qb, ta, tb = wq[ia], wq[ib], wt[ia], wt[ib]
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
seq, lab, seeds, seed_lab = [qa], [f"walk point t={ta:.3f} sec: " + ("valid" if d.module.validate(qa.astype(np.float32), d.env, True) else "COLLIDES")], [qa], ["walk point (start)"]
for k in range(8):
    f = (k + 1) / 8; seed = qa + (qb - qa) * f; ti = ta + (tb - ta) * f
    seeds.append(seed); seed_lab.append(f"unprojected lane {k+1}, t={ti:.4f} sec: " + ("valid" if d.module.validate(seed.astype(np.float32), d.env, True) else "COLLIDES"))
    try: o = np.asarray(d.module.project(np.clip(seed, lo, hi).astype(np.float32), [d.region_at(setup, ti), setup.com], d.make_settings()), float)
    except ValueError: o = None
    if o is None: seq.append(seed); lab.append(f"lane {k+1}: projection FAILED (showing the unprojected lane)"); continue
    ok = d.module.validate(o.astype(np.float32), d.env, True)
    seq.append(o); lab.append(f"lane {k+1}, t={ti:.4f} sec, moved {np.linalg.norm(o-seed):.3f} j by the projection: " + ("valid" if ok else "COLLIDES " + "; ".join(d.pair_names(setup, o))))
seq.append(qb); lab.append(f"walk point t={tb:.3f} sec: " + ("valid" if d.module.validate(qb.astype(np.float32), d.env, True) else "COLLIDES"))
seeds.append(qb); seed_lab.append("walk point (end)")
for l in lab: print(l)
if "--check" in sys.argv: sys.exit()
viewer = d.Viewer(setup); viewer.set_q(qa)
viewer.add_trace("lanes", np.array(seq))
viewer.add_sequence("Walk edge", seq, lab)
viewer.add_sequence("Unprojected line", seeds, seed_lab)
viewer.add_sequence("Walk path", list(wq), [f"walk t={t:.2f} sec" for t in wt])
print("\nviser running, Ctrl-C to quit")
while True: time.sleep(1.0)
