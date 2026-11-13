"""One of the 3 planner failures (argv: dataset, sid): exact walk vs C++ planner vs Python planner copy, with why each stops."""
import contextlib, io, sys, json
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d
import debug_rby1_pyplanner as pp
from diffik_benchmark import STARTS_DIR, TRAJECTORIES_DIR, transforms as tf
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams
from diffik_benchmark.starts import StartSet, starts_path
from diffik_benchmark.trajectories import iter_episodes

ds, sid = sys.argv[1], int(sys.argv[2]); N, STRIDE = 31, 6
ITERS = int(sys.argv[3]) if len(sys.argv) > 3 else 5000
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
print(f"{ds} sid {sid}: start valid {d.module.validate(setup.q_start.astype(np.float32), d.env, True)}")
path, rej = d.walk_reference(setup, "BoundedLS")
print(f"exact walk: reaches t={path[-1][0]:.2f}, stalled={bool(rej)}")
r = d.plan_with(setup, 23.0, range_=1.0)
print(f"C++ planner (halton): solved={r.solved} iterations={getattr(r,'iterations',None)} nodes={getattr(r,'size',None)}")
lo, hi = np.asarray(d.module.lower_bounds(), float), np.asarray(d.module.upper_bounds(), float)
w = np.ones(23); w[:2] = np.max((hi - lo)[3:]) / (hi[0] - lo[0])
pl = pp.PyPlanner(setup, 23.0, range_=1.0, accept="t", qweights=w)
ok = pl.run(ITERS, (1, 10, 100, 1000, 5000, 20000))
print(f"python planner: solved={ok}")
for it, size, tmax in pl.history: print(f"  {it:>6} -> {size:>5} nodes  t_max={tmax:.3f}")
print("explore:", dict(pl.explore_log.most_common())); print("walk:", dict(pl.walk_log.most_common()))
Tm = max(pl.T); print("tree t_max", Tm, "node", int(np.argmax(pl.T)))
for node, c in sorted(pl.fail_nodes.items(), key=lambda kv: -sum(kv[1].values()))[:5]:
    print(f"  node {node}: t={pl.T[node]:.3f}, {dict(c.most_common(3))}")

# --- the frontier: nodes with t > 0.9 and what the goal walk does from the best one
top = np.argsort(pl.T)[::-1][:3]
print("\nfrontier nodes (t):", [(int(i), round(pl.T[i], 3)) for i in top], " tree nodes with t>0.9:", int(np.sum(np.array(pl.T) > 0.9)))
i = int(top[0]); q, t = pl.Q[i].astype(np.float64), pl.T[i]
err, prop = pl.goal(q); delta = np.asarray(prop, float) - q; dt = 1 - t
norm = float(np.sqrt(np.sum((pl.qw * delta) ** 2) + (pl.w * dt) ** 2)); cl = min(1.0, pl.range / norm)
print(f"node {i}: end-pose error {err:.4g}, |step| {norm:.3f}, clamp {cl:.3f}, dt to 1: {dt:.3f}, max |dq| {np.abs(delta).max():.3f} (joint {int(np.abs(delta).argmax())})")
status, joints, ts, info = pl.steer(i, q + delta * cl, t + dt * cl, min(norm, pl.range))
print("goal step status", status, {k: v for k, v in info.items() if k != "projection"}, info.get("projection"))
# same step but with shrinking size
for f in (1.0, 0.5, 0.25, 0.1, 0.05, 0.02):
    c = cl * f; status, joints, ts, info = pl.steer(i, q + delta * c, t + dt * c, min(norm, pl.range) * f)
    print(f"  step x{f}: status {status} {info['failure']} t_end {ts[-1] if ts else None}")

print("\nstep scan from the frontier node: factor, status, failure, chord, gap, allowed gap")
for f in (1.0, 0.7, 0.5, 0.35, 0.25, 0.18, 0.12, 0.1, 0.07):
    c = cl * f; status, joints, ts, info = pl.steer(i, q + delta * c, t + dt * c, min(norm, pl.range) * f)
    print(f"  x{f:<5} t+{dt*c:.4f} status {status} {info['failure']:<18} chord {info['trace_chord']:.4f} gap {info['trace_gap']:.4f} allowed {info['trace_allowed_gap']:.4f}")

print("\ntolerance", pl.tol, "| end-pose error at frontier", err)
pa = pp.PyPlanner(setup, 23.0, range_=1.0, accept="t", qweights=w, walk_mode="advance")
print("python planner, walk 'advance' (halve the t step on failure):", pa.run(ITERS, (1, 10, 100, 1000)), "iterations", pa.iterations, "walk:", dict(pa.walk_log))

ps = pp.PyPlanner(setup, 23.0, range_=1.0, accept="t", qweights=w, shrink_on="trace_continuity")
print("python planner, LM goal step halved only after trace_continuity:", ps.run(ITERS, (1, 10, 100, 1000)), "iterations", ps.iterations, "walk:", dict(ps.walk_log))

# ---- the failing full goal step, stage by stage, with units (j = joint units incl. base, mr = SE3, sec = curve parameter t)
print("\n==== the failing full goal step from node X (t_X = %.4f sec)" % t)
X = q.copy(); tB = float(t + dt * cl); target = X + delta * cl
print(f"stage 0: target joints = X + LM step; |target - X| = {np.linalg.norm(delta*cl):.4f} j ; target t = {tB:.4f} sec (t advance {tB - t:.4f} sec)")
def pr(qq, tt):
    cs = [d.region_at(setup, tt), setup.com]
    try: return np.asarray(d.module.project(np.clip(qq, lo, hi).astype(np.float32), cs, d.make_settings()), float)
    except ValueError: return None
Xp = pr(target, tB)
print(f"stage 1: project target onto pose Y(t={tB:.4f} sec): converged={Xp is not None}")
if Xp is not None:
    chord = float(np.linalg.norm(Xp - X)); spacing = chord / 8
    print(f"  X' = projected endpoint; chord |X' - X| = {chord:.4f} j  -> lane spacing = chord/8 = {spacing:.4f} j, allowed gap = 2*spacing = {2*spacing:.4f} j")
    prev, tprev, Yprev = X, t, np.array(d.pose_at(setup, t))
    print("  lane i: t_i [sec] | reference moves Y_(i-1)->Y_i [mr: pos m, rot rad] | projected lane moves from previous lane [j] (allowed above)")
    for k in range(8):
        f = (k + 1) / 8; qi = X + (Xp - X) * f; ti = t + (tB - t) * f
        o = pr(qi, ti); Y = np.array(d.pose_at(setup, ti))
        dpos = np.linalg.norm(Y[:, 4:] - Yprev[:, 4:], axis=1); drot = 2 * np.arccos(np.clip(np.abs(np.sum(Y[:, :4] * Yprev[:, :4], axis=1)), 0, 1))
        print(f"  lane {k+1}: t={ti:.4f} | pos {np.round(dpos,5).tolist()} m, rot {np.round(drot,5).tolist()} rad | " + ("projection failed" if o is None else f"{np.linalg.norm(o - prev):.4f} j"))
        if o is not None: prev = o
        tprev, Yprev = ti, Y
if Xp is not None:
    print(f"\ncheck: |X' - target| = {np.linalg.norm(Xp - target):.6f} j ; squared SE3 error to the end pose Y(1): target {pl.goal(target)[0]:.3g}, X' {pl.goal(Xp)[0]:.3g}, X {pl.goal(X)[0]:.3g}")

# ---- what each lane's projection does (stage 2), per solver
from scipy.spatial.transform import Rotation as Rot
jn = list(d.module.joint_names())
def se3_err(qq, tt):
    Y = np.array(d.pose_at(setup, tt)); out = []
    for e in range(2):
        T = np.asarray(d.module.eefk(np.asarray(qq, np.float32), e)); dp = np.linalg.norm(T[:3, 3] - Y[e, 4:])
        R0 = Rot.from_quat([Y[e, 1], Y[e, 2], Y[e, 3], Y[e, 0]]).as_matrix(); ang = np.linalg.norm(Rot.from_matrix(T[:3, :3] @ R0.T).as_rotvec())
        out.append((dp, ang))
    return out
print("\n==== lane probe (X, X' as above). seed = unprojected lane. columns: seed error to Y_i [m, rad per EE] | moved by projection [j] | error after | top moved joints [j]")
for method in ("BoundedLS", "InnerLM"):
    st_ = d.make_settings(method); print(f"-- solver {method}")
    prev = X
    for k in range(8):
        f = (k + 1) / 8; seed = X + (Xp - X) * f; ti = t + (tB - t) * f
        try: o = np.asarray(d.module.project(np.clip(seed, lo, hi).astype(np.float32), [d.region_at(setup, ti), setup.com], st_), float)
        except ValueError: print(f"  lane {k+1}: projection failed"); continue
        e0, e1 = se3_err(seed, ti), se3_err(o, ti); mv = np.abs(o - seed); top = np.argsort(mv)[::-1][:3]
        print(f"  lane {k+1}: seed err {[(round(a,4), round(b,4)) for a, b in e0]} | moved {np.linalg.norm(o-seed):.4f} j | after {[(round(a,5), round(b,5)) for a, b in e1]} | " + ", ".join(f"{jn[j]} {o[j]-seed[j]:+.3f}" for j in top) + f" | from prev lane {np.linalg.norm(o-prev):.4f} j")
        prev = o

# ======== why the exact walk gets past t~0.72 and the planner tree does not
print("\n==== walk path vs planner tree")
wt = np.array([p[0] for p in path]); wq = np.array([p[1] for p in path])
def wmet(a, b): return np.linalg.norm((a - b) * w, axis=-1)
Tt = np.array(pl.T); Qt = np.array(pl.Q, float)
fr = [k for k in np.argsort(Tt)[::-1][:200]]
print(f"planner t_max {Tt.max():.3f} sec. For frontier nodes, distance [j, weighted] to the walk path point at the same t (nearest sample):")
for k in fr[:3]:
    j = int(np.argmin(np.abs(wt - Tt[k]))); dq = Qt[k] - wq[j]
    top = np.argsort(np.abs(dq))[::-1][:4]
    print(f"  node {k} t={Tt[k]:.3f}: |q_node - q_walk(t={wt[j]:.3f})| = {wmet(Qt[k], wq[j]):.3f} j ; top joints " + ", ".join(f"{jn[i]} {dq[i]:+.2f}" for i in top))
# the walk path just past the stall
for tq in (0.70, 0.72, 0.74, 0.76, 0.80):
    j = int(np.argmin(np.abs(wt - tq))); print(f"  walk path t={wt[j]:.2f}: step from previous sample {wmet(wq[j], wq[j-1]):.3f} j")
# nearest tree node (all t<=0.72) to the walk path at each t, in joint metric: how far is the tree from the walk path?
for tq in (0.5, 0.6, 0.7, 0.72):
    j = int(np.argmin(np.abs(wt - tq))); m_ = np.abs(Tt - wt[j]) < 0.02
    print(f"  at t~{tq}: tree nodes within 0.02 sec: {int(m_.sum())}; nearest to walk path: " + (f"{wmet(Qt[m_], wq[j]).min():.3f} j" if m_.any() else "none"))
# steer from the best frontier node to the walk path's point 0.02 sec later (and a few later)
k = int(np.argmax(Tt))
for dtq in (0.02, 0.05, 0.1):
    j = int(np.argmin(np.abs(wt - (Tt[k] + dtq)))); dist = float(wmet(Qt[k], wq[j]) ** 2 + (23 * (wt[j] - Tt[k])) ** 2) ** 0.5
    st_, jo, ts_, info = pl.steer(k, wq[j].astype(np.float32), float(wt[j]), dist)
    print(f"  steer node {k}(t={Tt[k]:.3f}) -> walk path t={wt[j]:.3f}, distance {dist:.2f}: status {st_} {info['failure']} step_length {info['step_length']:.2f} drift {info['drift']:.3f}")
# restart the planner from the walk path's configuration at t0
for t0 in (0.60, 0.70, 0.72, 0.74, 0.80):
    j = int(np.argmin(np.abs(wt - t0))); r2 = d.plan_with(setup, 23.0, range_=1.0, q_start=wq[j].astype(np.float32), start_t=float(wt[j]))
    print(f"  C++ planner started AT the walk path (t={wt[j]:.2f} sec): solved={r2.solved} iterations={getattr(r2,'iterations',None)}")

print("\n==== steering along the walk path itself, node = walk path point (the planner's steer, not the walk)")
for ia in (int(np.argmin(np.abs(wt - 0.70))), int(np.argmin(np.abs(wt - 0.72)))):
    ca = Qt.shape  # noqa
    pl2 = pp.PyPlanner(setup, 23.0, range_=1.0, accept="t", qweights=w, q_start=wq[ia].astype(np.float32), t_start=float(wt[ia]))
    for step in (1, 2, 3, 5, 8):
        ib = min(ia + step, len(wt) - 1); dist = float(wmet(wq[ia], wq[ib]) ** 2 + (23 * (wt[ib] - wt[ia])) ** 2) ** 0.5
        st_, jo, ts_, info = pl2.steer(0, wq[ib].astype(np.float32), float(wt[ib]), dist)
        print(f"  from walk t={wt[ia]:.2f} to walk t={wt[ib]:.2f} (+{wt[ib]-wt[ia]:.2f} sec, {wmet(wq[ia], wq[ib]):.3f} j, distance {dist:.3f}): status {st_} {info['failure']} winner {info.get('winner_lane')} drift {info['drift']:.3f}/{info['drift_limit']:.2f} chord {info['trace_chord']:.3f} gap {info['trace_gap']:.3f}/{info['trace_allowed_gap']:.3f} " + str(info['projection']['exit']))

print("\n==== the trace of walk t=0.72 -> 0.74 (the walk's own next point), lane by lane, 8 lanes as in trace()")
ia = int(np.argmin(np.abs(wt - 0.72))); ib = ia + 1; qa, qb, ta, tb = wq[ia], wq[ib], wt[ia], wt[ib]
print(f"  t {ta:.3f} -> {tb:.3f} sec, {wmet(qa, qb):.3f} j; walk accepted this edge; validity of both ends: {d.module.validate(qa.astype(np.float32), d.env, True)} {d.module.validate(qb.astype(np.float32), d.env, True)}")
for k in range(8):
    f = (k + 1) / 8; seed = qa + (qb - qa) * f; ti = ta + (tb - ta) * f
    try: o = np.asarray(d.module.project(np.clip(seed, lo, hi).astype(np.float32), [d.region_at(setup, ti), setup.com], d.make_settings()), float)
    except ValueError: print(f"  lane {k+1}: projection failed"); continue
    ok = d.module.validate(o.astype(np.float32), d.env, True); ok_seed = d.module.validate(seed.astype(np.float32), d.env, True)
    print(f"  lane {k+1}: t={ti:.4f} projected moved {np.linalg.norm(o-seed):.4f} j from the line; valid {ok}" + ("" if ok else f" -> colliding: {d.pair_names(setup, o)}"))

print("\n==== exact walk with a finer edge check")
for ks in (0.1, 0.03, 0.01):
    d.MAX_KNOT_STEP = ks; p2, r2 = d.walk_reference(setup, "BoundedLS")
    print(f"  edge check step {ks} j: walk reaches t={p2[-1][0]:.2f} sec, stalled={bool(r2)}" + (f"; rejected there: {sorted({x[0] for x in r2})}" if r2 else ""))
