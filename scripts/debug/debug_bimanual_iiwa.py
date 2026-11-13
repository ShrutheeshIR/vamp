"""Browse one failing bimanual-iiwa curve-planner instance in viser: the exact reference, a greedy walk that
tracks it with BoundedLS, and, where the walk stalls, every candidate it rejected and why.

An instance is one plan of a (3,3) chain: start id SID of the bimanual_cloth_folding start set, the 16-target window's plan K
(plan 0 starts from the dataset start; plan K > 0 replays the chain with vamp_curve_rrtc to get the plan's start q, which
takes a planner call per earlier plan). The model drawn is VAMP's (spheres), the one the planner and the walk validate in.

Frames of the slider: the walk's configurations (t = 0 .. stall), then the rejected candidates at the stall
(no convergence is not drawable; invalid ones are drawn, colliding spheres in red, the colliding pairs and joints at a
limit listed). The reference is drawn as a polyline per arm plus a triad at pose(t).

Run in the diff-ik-benchmarks container (vamp built with bimanual_iiwa_tool):
    python3 -u scripts/debug_bimanual_iiwa.py --sid 18 [--plan 0] [--starts 6]   # viser on :8080
    python3 -u scripts/debug_bimanual_iiwa.py --sid 18 --check                   # print the stall summary and exit
--starts K adds K random-restart starts at t=0 (walkers "random 1..K") next to the plan's own start.
"""
import sys

sys.path.insert(0, "/src/diffik-benchmark/scratch/vamp_bi")  # the bimanual_iiwa_tool build
sys.meta_path[:] = [f for f in sys.meta_path if "ditable" not in repr(f)]
import collections
import time
import xml.etree.ElementTree as ET

import numpy as np
import vamp
import yourdfpy
from diffik_benchmark import STARTS_DIR, TRAJECTORIES_DIR, transforms as tf
from diffik_benchmark.backend import DrakeBackend
from diffik_benchmark.methods import make_method
from diffik_benchmark.methods.vamp_curve_rrtc import (_CurveRobot, VampCurveRRTCParams, pad_keyframes_multi,
                                                      shared_parameters, clip_inside_limits)
from diffik_benchmark.robots import get_robot
from diffik_benchmark.scoring import ScorerParams, score_segment
from diffik_benchmark.starts import StartSet, starts_path
from diffik_benchmark.trajectories import iter_episodes

URDF = "/src/cricket/resources/bimanual_iiwa_tool/iiwa_bimanual_spherized.urdf"
DS, STRIDE, N_PLAN, STEP = "bimanual_cloth_folding", 6, 3, 0.02


def arg(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


SID, PLAN, N_RAND, CHECK = int(arg("--sid", 18)), int(arg("--plan", 0)), int(arg("--starts", 6)), "--check" in sys.argv
print(f"instance: --sid {SID} --plan {PLAN} (defaults 18 / 0), --starts {N_RAND}", flush=True)

spec = get_robot("iiwa_bimanual")
b = DrakeBackend(spec)
P = VampCurveRRTCParams()
r = _CurveRobot(b, P)
m = r.module
q_lo, q_hi = b.position_limits()
lo, hi = np.asarray(m.lower_bounds(), float), np.asarray(m.upper_bounds(), float)
joint_names = list(m.joint_names())
sp, tool_ref = ScorerParams(), spec.tool_ref_poses()


def bounded(base):
    """A copy of the planner's constraint settings with BoundedLS (joint limits as bounds, not a clamp)."""
    s = type(base)()
    s.tolerance, s.max_iterations, s.method = base.tolerance, base.max_iterations, vamp.ProjMethod.BoundedLS
    return s


stt, es = bounded(r.start_settings), bounded(r.settings)

# --- the instance: plan PLAN of the chain at start SID -----------------------------------------------------
st = StartSet.load(starts_path("iiwa_bimanual", DS, STARTS_DIR))
ep = {int(e.index): e for e in iter_episodes(TRAJECTORIES_DIR / DS, include_flagged=True)}[int(st.episode_index[SID])]
method = make_method("vamp_curve_rrtc", {"segment": "false"})
q0, s0 = np.asarray(st.q[SID], float).copy(), int(st.start_index[SID])


def segment(s):
    return ep.segment(s, N_PLAN, STRIDE).transformed(st.X_W_ref[SID], tool_ref).with_tolerance(sp.eps_pos, sp.eps_rot)


for k in range(PLAN):  # replay the chain: each plan starts where the previous one ended
    seg = segment(s0)
    score = score_segment(b, seg, method(b, q0, seg), sp)
    if not score.success:
        sys.exit(f"plan {k} of the chain already fails; the instance does not have a plan {PLAN}")
    q0, s0 = score.final_q, s0 + STRIDE * N_PLAN
seg = segment(s0)

qplan = clip_inside_limits(q0, q_lo, q_hi, P.start_margin)
pos, quat = seg.pos[:, r.ee_order].copy(), tf.quat_normalize(seg.quat[:, r.ee_order])
ap, aq = b.fk(qplan)
pos[0], quat[0] = ap[r.ee_order], tf.quat_normalize(aq[r.ee_order])
for e in range(2):
    if np.dot(quat[0, e], quat[1, e]) < 0:
        quat[0, e] = -quat[0, e]
kp, kq, _ = pad_keyframes_multi(pos, quat, r.n_points, P.rotation_weight)
taus = np.asarray(shared_parameters(kp, kq, P.rotation_weight), float)
q_start = np.asarray(r.project_start(qplan, pos[0], quat[0]), np.float32)[r.to_vamp].astype(float)
curve = m.LerpSO3R3MultiConstraint(kp.transpose(1, 0, 2).tolist(), kq.transpose(1, 0, 2).tolist(), P.rotation_weight, False)
curve.set_parameters([float(v) for v in taus])
env = r.env


def pose_at(t):
    """[[qw qx qy qz x y z]] for the left and right tool at curve parameter t (VAMP end-effector order)."""
    i = int(np.clip(np.searchsorted(taus, t, side="right") - 1, 0, len(taus) - 2))
    u = (t - taus[i]) / (taus[i + 1] - taus[i])
    return [[float(v) for v in (*tf.slerp(kq[i, e], kq[i + 1, e], u), *(kp[i, e] + u * (kp[i + 1, e] - kp[i, e])))] for e in range(2)]


def region_at(t):
    return m.TaskSpaceConstraint([[1, 0, 0, 0, 0, 0, 0]] * 2, pose_at(t), [[0.0] * 6] * 2, [[0.0] * 6] * 2)


# --- names of the model's spheres (matched to the URDF's, as in survey_bi.py) ------------------------------
urdf = yourdfpy.URDF.load(URDF, load_meshes=False, build_scene_graph=True)
geoms = []
for link in ET.parse(URDF).getroot().findall("link"):
    for col in link.findall("collision"):
        org = col.find("origin")
        if col.find("geometry").find("sphere") is not None:
            xyz = np.array([float(x) for x in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
            geoms.append((link.get("name"), xyz))
rng = np.random.default_rng(0)
qs_ = [np.clip(rng.uniform(-0.8, 0.8, 14), lo, hi) for _ in range(3)]


def urdf_centers(q):
    urdf.update_cfg(q)
    return np.array([urdf.get_transform(l, "world")[:3, :3] @ o + urdf.get_transform(l, "world")[:3, 3] for l, o in geoms])


UC = [urdf_centers(q) for q in qs_]
VC = [np.array([[s.x, s.y, s.z] for s in m.fk(q.astype(np.float32))]) for q in qs_]
names = [geoms[int(np.argmin([sum(np.linalg.norm(VC[k][i] - UC[k][j]) for k in range(3)) for j in range(len(geoms))]))][0]
         for i in range(len(VC[0]))]
radii = [float(s.r) for s in m.fk(qs_[0].astype(np.float32))]


# --- the walk ----------------------------------------------------------------------------------------------
def proj(seed, reg):
    try:
        return np.asarray(m.project(np.clip(seed, lo, hi).astype(np.float32), [reg], stt), float)
    except ValueError:
        return None


def pairs_of(q):
    return sorted({" <-> ".join(sorted((names[a], names[c]))) for a, c in m.debug(q.astype(np.float32), env)[1]})


def classify(q_from, t, o, t2):
    """Why candidate o (a projection onto pose(t2)) is rejected, or None if the walk can take it."""
    if o is None:
        return "no convergence"
    if not m.validate(o.astype(np.float32), env, True):
        return "self-collision" if m.validate(o.astype(np.float32), env, False) else "joint limit"
    d = m.interpolate_curve_path([q_from.astype(np.float32), o.astype(np.float32)], [float(t), float(t2)], env, [curve], es, 0.01)
    return None if d.interpolated else "edge check"


def walk(q, t, rg, n_cand=60):
    """Greedy exact tracking: per step of 0.02 in t, the first of n_cand seeds (the config, then noisy copies) whose projection is
    valid and whose edge passes. Returns the path [(t, q)] and, if stalled, the stall [(reason, q or None)] of the last step."""
    path = [(t, q)]
    while t < 1 - 1e-9:
        t2, reg, rejected = min(1.0, t + STEP), region_at(min(1.0, t + STEP)), []
        for k in range(n_cand):
            seed = q if k == 0 else q + (0.03 if k < 20 else 0.1 if k < 40 else 0.3) * rg.standard_normal(14)
            o = proj(seed, reg)
            why = classify(q, t, o, t2)
            if why is None:
                q, t = o, t2
                path.append((t, q))
                break
            rejected.append((why, o))
        else:
            return path, rejected
    return path, []


def random_starts(n, seed=0):
    rg, out, reg0 = np.random.default_rng(seed), [], region_at(0.0)
    for _ in range(600):
        o = proj(rg.uniform(lo, hi), reg0)
        if o is not None and m.validate(o.astype(np.float32), env, True):
            out.append(o)
            if len(out) == n:
                break
    return out


starts = {"plan start": q_start}
for i, o in enumerate(random_starts(N_RAND), 1):
    starts[f"random {i}"] = o
results = {}


def run(name):
    if name not in results:
        t0 = time.time()
        results[name] = walk(starts[name], 0.0, np.random.default_rng(1))
        path, rej = results[name]
        print(f"[walk] {name}: reaches t={path[-1][0]:.2f}" + (f", stalls ({len(rej)} candidates rejected: {dict(collections.Counter(w for w, _ in rej))})"
                                                                if rej else " (end)") + f" in {time.time() - t0:.0f} s", flush=True)
    return results[name]


print(f"sid {SID} plan {PLAN}: start valid in VAMP: {bool(m.validate(q_start.astype(np.float32), env, True))}; "
      f"{len(starts)} walkers", flush=True)
if CHECK:
    for name in starts:
        run(name)
    sys.exit(0)

# --- viser ------------------------------------------------------------------------------------------------
import viser

server = viser.ViserServer()
server.scene.add_grid("/grid", width=3.0, height=3.0, plane="xy")
server.scene.add_frame("/world", axes_length=0.1, axes_radius=0.003)
GREY, RED, BLUE, GREEN = (120, 150, 200), (230, 40, 40), (90, 140, 230), (90, 190, 120)
sph = [server.scene.add_icosphere(f"/robot/sphere_{i}", radius=radii[i], color=GREY, opacity=0.5, position=(0.0, 0.0, 0.0))
       for i in range(len(names))]
for e, col in enumerate((BLUE, GREEN)):  # reference polyline per tool, and the triad at pose(t)
    pts = kp[:, e]
    server.scene.add_line_segments(f"/ref/line_{e}", points=np.stack([pts[:-1], pts[1:]], axis=1), colors=col, line_width=3.0)
    server.scene.add_icosphere(f"/ref/start_{e}", radius=0.012, color=col, position=tuple(pts[0]))
tri = [server.scene.add_frame(f"/ref/pose_{e}", axes_length=0.08, axes_radius=0.004) for e in range(2)]
tool = [server.scene.add_frame(f"/tool/{e}", axes_length=0.05, axes_radius=0.003) for e in range(2)]

with server.gui.add_folder("Walk"):
    walker = server.gui.add_dropdown("start", list(starts))
    slider = server.gui.add_slider("frame", min=0, max=1, step=1, initial_value=0)
    prev_btn, next_btn = server.gui.add_button("prev"), server.gui.add_button("next")
    summary = server.gui.add_text("walk", "", disabled=True)
    frame_text = server.gui.add_text("frame info", "", disabled=True)
    pairs_text = server.gui.add_text("colliding pairs", "", disabled=True)
    limit_text = server.gui.add_text("joints near a limit", "", disabled=True)
    err_text = server.gui.add_text("tool vs pose(t)", "", disabled=True)
with server.gui.add_folder("Joints (slider range = joint limits)"):
    jsl = [server.gui.add_slider(joint_names[j].replace("iiwa_", "").replace("_joint_", " j"), min=float(lo[j]), max=float(hi[j]),
                                 step=1e-4, initial_value=float(np.clip(q_start[j], lo[j], hi[j])))
           for j in range(14)]
with server.gui.add_folder("Joint tools"):
    proj_cur_btn = server.gui.add_button("project sliders to CURRENT point pose(t) (BoundedLS)")
    proj_next_btn = server.gui.add_button("project sliders to NEXT point pose(t + 0.02) (BoundedLS)")
    proj_text = server.gui.add_text("projection", "", disabled=True)
    q_text = server.gui.add_text("q (select + copy)", "")
    print_btn = server.gui.add_button("print q to the terminal")
with server.gui.add_folder("Display"):
    opacity = server.gui.add_slider("sphere opacity", min=0.05, max=1.0, step=0.05, initial_value=0.5)

frames = []  # (label, t, q or None, reason or None)


def build_frames():
    path, rej = run(walker.value)
    frames[:] = [(f"walk, t={t:.2f}", t, q, None) for t, q in path]
    t_stall = path[-1][0]
    frames.extend((f"rejected at the next step (t {t_stall:.2f} -> {min(1.0, t_stall + STEP):.2f}): {w}", t_stall, o, w)
                  for w, o in rej if o is not None)
    c = collections.Counter(w for w, _ in rej)
    summary.value = (f"reaches t=1" if not rej else f"STALLS at t={t_stall:.2f}; of {len(rej)} candidates: "
                     + ", ".join(f"{k} {v}" for k, v in c.most_common()))
    slider.max = max(len(frames) - 1, 1)
    slider.value = len(path) - 1


cur_t, next_t = [0.0], [STEP]  # the frame's t, and t + STEP: the two projection targets
busy = [False]  # set while show() writes the joint sliders, so their callbacks do not re-enter


def show(i, q_manual=None):
    """Draw frame i; with q_manual (the joint sliders moved by hand) draw that configuration at frame i's t instead."""
    label, t, q, why = frames[i]
    if q_manual is not None:
        q, label = q_manual, "manual (joint sliders)"
    q = np.asarray(q, float)
    qf = q.astype(np.float32)
    hit = {j for a, c in m.debug(qf, env)[1] for j in (a, c)}
    for j, c in enumerate(m.fk(qf)):
        sph[j].position = (c.x, c.y, c.z)
        sph[j].color = RED if j in hit else GREY
        sph[j].opacity = max(opacity.value, 0.7) if j in hit else opacity.value
    # reference at t (the next pose for a rejected candidate), and where the tools actually are
    t_ref = min(1.0, t + STEP) if why else t
    for e, pz in enumerate(pose_at(t_ref)):
        tri[e].wxyz, tri[e].position = tuple(pz[:4]), tuple(pz[4:])
    ee = np.asarray(m.eefk(qf))
    perr = []
    for e in range(2):
        pe = np.asarray(ee[e]).ravel() if np.ndim(ee) > 1 else np.asarray(ee).ravel()
        perr.append(np.linalg.norm(pe[-3:] - np.asarray(pose_at(t_ref)[e][4:])) * 1000 if pe.size >= 3 else float("nan"))
    cur_t[0], next_t[0] = t, min(1.0, t + STEP)
    q_text.value = "[" + ", ".join(f"{v:.6f}" for v in q) + "]"
    err_text.value = f"position error left {perr[0]:.1f} mm, right {perr[1]:.1f} mm (to pose({t_ref:.2f}))"
    margin = np.minimum(q - lo, hi - q)
    if q_manual is None:
        busy[0] = True
        for j, h in enumerate(jsl):
            h.value = float(np.clip(q[j], lo[j], hi[j]))
        busy[0] = False
    frame_text.value = label
    pairs_text.value = "; ".join(pairs_of(q)) or "none"
    limit_text.value = "; ".join(f"{joint_names[j].replace('iiwa_', '')} {margin[j]:.4f}" for j in np.argsort(margin)[:3]
                                 if margin[j] < 0.05) or "none within 0.05 rad"


def set_sliders(q):
    busy[0] = True
    for j, h in enumerate(jsl):
        h.value = float(np.clip(q[j], lo[j], hi[j]))
    busy[0] = False


def project_sliders(t_target, name):
    q = np.array([h.value for h in jsl], float)
    o = proj(q, region_at(t_target))
    if o is None:
        proj_text.value = f"no convergence onto the {name} point pose({t_target:.2f}) from these joint values"
        return
    set_sliders(o)
    show(int(slider.value), o)
    proj_text.value = (f"converged onto the {name} point pose({t_target:.2f}); |dq| = {np.linalg.norm(o - q):.3f} rad; "
                       f"valid in VAMP: {bool(m.validate(o.astype(np.float32), env, True))}")


@proj_cur_btn.on_click
def _(_):
    project_sliders(cur_t[0], "current")


@proj_next_btn.on_click
def _(_):
    project_sliders(next_t[0], "next")


@print_btn.on_click
def _(_):
    print("q =", q_text.value, flush=True)


def on_joint(_):
    if not busy[0]:
        show(int(slider.value), np.array([h.value for h in jsl], float))


for h in jsl:
    h.on_update(on_joint)


@walker.on_update
def _(_):
    build_frames()
    show(int(slider.value))


@slider.on_update
def _(_):
    show(int(slider.value))


@opacity.on_update
def _(_):
    show(int(slider.value))


@prev_btn.on_click
def _(_):
    slider.value = max(0, int(slider.value) - 1)


@next_btn.on_click
def _(_):
    slider.value = min(len(frames) - 1, int(slider.value) + 1)


build_frames()
show(int(slider.value))
print("viser running: dropdown picks the walker (random starts walk when first selected); slider steps the walk, then the "
      "rejected candidates at the stall. Ctrl-C to quit", flush=True)
while True:
    time.sleep(1.0)
