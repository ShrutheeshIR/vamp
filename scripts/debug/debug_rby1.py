"""Debug the VAMP curve planner (`vamp_curve_rrtc`) on one RB-Y1 problem.

What it does, in order (each step is a function below; switch steps in CONFIG):
  1. loads one window of the hard RB-Y1 set and VAMP's model, and checks the start in VAMP's model;
  2. TSR chain: project the previous configuration onto each waypoint in turn (what `vamp_tsr_ik` does);
  3. planner: runs `task_space_rrtc_curve` with the settings in CONFIG, once per parameter weight;
  4. walk: a TSR chain in steps of 0.02 in t, with up to 60 seeds per step, VAMP validity, the planner's
     edge check and the CoM constraint; prints why it stalls if it does;
  5. baseline: the QP's trajectory for the same window, checked knot by knot in VAMP's model;
  6. viser: the robot, the reference, joint sliders and one slider each over the plan, the walk and the baseline.

Default problem: start 26, N = 31 (32 poses). The QP and pink-pf4 pass the scorer on it, the planner does not.

Run in the diff-ik-benchmarks container (vamp built with VAMP_ROBOTS=rby1_tool):
    python3 -u scripts/debug_rby1.py
"""
import collections
import json
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np
import viser
import yourdfpy
from viser import uplot
from viser.extras import ViserUrdf

sys.path.insert(0, "/src/diffik-benchmark/src")
import vamp  # noqa: E402
from diffik_benchmark import transforms as tf  # noqa: E402
from diffik_benchmark.methods.vamp_curve_rrtc import (  # noqa: E402
    VampCurveRRTCParams, pad_keyframes_multi, shared_parameters, support_polygon,
)
from diffik_benchmark.trajectories import Episode  # noqa: E402

# =====================================================================================================
# CONFIG
# =====================================================================================================
P = VampCurveRRTCParams()  # the benchmark method's defaults; override below to experiment

# --- the problem: (start id, N, stride); N targets beyond the start give N + 1 poses ---
DATASET = "nymeria_L60x180_v01"
PROBLEMS = [
    (1, 16, 6), (4, 16, 6), (5, 16, 6), (8, 16, 6),  # 0-3: N = 16, valid starts where the planner failed
    (26, 31, 6),                                     # 4: 32 poses; QP and pink-pf4 pass the scorer, the planner does not
]
PROBLEM_INDEX = 4

# --- planner knobs (used by plan_with) ---
RANGE = 0.25                         # longest extension of the tree (joint metric)
PARAMETER_WEIGHTS = (P.parameter_weight, 1.0)  # weight of t in the metric; one planner run per entry
N_TRIALS = 3                         # runs per weight (the sampler is Halton, so runs are identical unless seeded)
MAX_GREEDY_STEPS = 10  # steps of the goal-directed walk per extension
PROJECT_SAMPLES = P.project_samples  # project random samples onto the curve
MONOTONE_T = True                    # forbid moving back in t
MAX_ITERATIONS = P.max_iterations
MAX_SAMPLES = 200_000                # capacity of the tree
STABILITY = True                     # CoM constraint in the plan and in the walk
STABILITY_INSET = P.stability_inset
WALK_BY_PARAMETER = False            # the goal walk accepts a step if t advances (not if the end-pose error falls)
BASE_METRIC_WEIGHT = False           # weight base x, y so their full range counts as the largest revolute range

# --- projection / curve knobs ---
ROT_WEIGHT = 1.0       # weight of rotation error in the curve constraint and in t's length
PROJ_METHOD = "BoundedLS"            # "InnerLM" (clamps joint limits) or "BoundedLS" (limits as bounds)
TOL = 1e-5                           # projection tolerance in the planner
MAX_KNOT_STEP = 0.1                  # step of the edge check (interpolate_curve_path)

# --- the walk (step 4) ---
WALK_STEP_T = 0.02                   # t per step
WALK_CANDIDATES = 60                 # seeds per step: the configuration, then noisy copies (std 0.03 / 0.1 / 0.3)
WALK_METHODS = ("BoundedLS", "InnerLM")

# --- which steps run ---
RUN_TSR_CHAIN = True
RUN_PLANNER = True
RUN_WALK = True
RUN_BASELINE = True
RUN_VIEWER = True

# --- paths and robot constants ---
DATA = "/src/diffik-benchmark/data"
URDF = "/src/cricket/resources/rby1_tool/rby1_primitive.urdf"  # the model VAMP was generated from (spheres + boxes + meshes)
BASE_RANGES = {"base/base_x": (-3.0, 3.0), "base/base_y": (-3.0, 3.0), "base/base_rz": (-2 * np.pi, 2 * np.pi)}
EE_NAMES = ["left_arm/tool", "right_arm/tool"]  # VAMP's end effectors; the benchmark's are (right, left)
EE_ORDER = [1, 0]                    # VAMP end effector i is the benchmark's EE_ORDER[i]
EE_COLORS = [(60, 120, 230), (240, 150, 40)]
GREY, RED = (120, 150, 200), (230, 40, 40)

module = vamp.rby1_tool
env = vamp.Environment()


# =====================================================================================================
# GEOMETRY HELPERS
# =====================================================================================================
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def matrix_to_wxyz(R):
    t = np.trace(R)
    if t > 0:
        r = np.sqrt(1.0 + t)
        w = 0.5 * r
        x, y, z = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * 0.5 / r
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
        v = np.zeros(3)
        v[i] = 0.5 * r
        v[j] = (R[j, i] + R[i, j]) * 0.5 / r
        v[k] = (R[k, i] + R[i, k]) * 0.5 / r
        w = (R[k, j] - R[j, k]) * 0.5 / r
        x, y, z = v
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


# =====================================================================================================
# PROBLEM AND MODEL
# =====================================================================================================
@dataclass
class Problem:
    """One window of the dataset, in VAMP's joint order and with the base recentred (as vamp_curve_rrtc does)."""
    start_id: int
    n: int
    stride: int
    episode: int
    start_index: int
    X_W_ref: np.ndarray
    q0: np.ndarray          # VAMP order, base at the origin
    q0_bench: np.ndarray    # the start set's q, benchmark order, unshifted (for running the baselines)
    base_shift: np.ndarray  # what was subtracted from q0 (base x, y)
    to_vamp: np.ndarray     # benchmark joint order -> VAMP's
    pos: np.ndarray         # (N + 1, 2, 3), VAMP end-effector order, recentred
    quat: np.ndarray        # (N + 1, 2, 4)


def load_problem(start_id, n, stride):
    starts = np.load(f"{DATA}/starts/rby1/{DATASET}.npz")
    meta = json.load(open(f"{DATA}/starts/rby1/{DATASET}.json"))
    bench_names = [x.split("/")[-1] for x in meta["joint_names"]]
    to_vamp = np.array([bench_names.index(x.split("/")[-1]) for x in module.joint_names()])
    q0_bench = np.asarray(starts["q"][start_id], float)
    q0 = q0_bench[to_vamp].copy()
    base_shift = np.zeros(len(q0))
    base_shift[:2] = q0[:2]
    q0 -= base_shift
    episode, start_index = int(starts["episode_index"][start_id]), int(starts["start_index"][start_id])
    X_W_ref = starts["X_W_ref"][start_id]
    seg = Episode.load(f"{DATA}/trajectories/{DATASET}/ep_{episode:05d}.npz").segment(start_index, n, stride).transformed(X_W_ref)
    shift3 = np.array([base_shift[0], base_shift[1], 0.0])
    print(f"start {start_id}: episode {episode}, start index {start_index}, N={n} ({n + 1} poses), stride {stride}")
    return Problem(start_id, n, stride, episode, start_index, X_W_ref, q0, q0_bench, base_shift, to_vamp,
                   seg.pos[:, EE_ORDER] - shift3, seg.quat[:, EE_ORDER].copy())


@dataclass
class Model:
    urdf: yourdfpy.URDF
    joint_names: list
    q_lo: np.ndarray       # slider range: VAMP's bounds, with the base's (infinite) ranges replaced
    q_hi: np.ndarray
    lo: np.ndarray         # VAMP's own bounds
    hi: np.ndarray
    sphere_geom: list      # (link, offset, radius), in the module's sphere order
    box_geom: list         # (link, offset, rotation, size), in the module's cuboid order
    names: list            # the names of the module.debug indices: spheres, then cuboids


def order_cuboids_like_vamp(box_geom, urdf):
    """The URDF's boxes in the module's cuboid order (the header orders them by link): matched by world centre
    at a few configurations."""
    n = len(box_geom)
    if n == 0 or not hasattr(module, "cuboid_fk"):
        return box_geom
    rng = np.random.default_rng(5)
    lo, hi = np.asarray(module.lower_bounds(), float), np.asarray(module.upper_bounds(), float)
    qs = []
    for _ in range(4):
        q = np.clip(rng.uniform(-0.8, 0.8, len(lo)), lo, hi)
        q[:3] = rng.uniform(-1, 1, 3)
        qs.append(q)
    urdf_centers, vamp_centers = [], []
    for q in qs:
        urdf.update_cfg(q)
        urdf_centers.append(np.array([urdf.get_transform(link, "world")[:3, :3] @ off + urdf.get_transform(link, "world")[:3, 3]
                                      for link, off, _, _ in box_geom]))
        vamp_centers.append(np.array([[c.x, c.y, c.z] for c in module.cuboid_fk(q.astype(np.float32))]))
    order = [int(np.argmin([sum(np.linalg.norm(vamp_centers[k][i] - urdf_centers[k][j]) for k in range(len(qs))) for j in range(n)]))
             for i in range(n)]
    assert sorted(order) == list(range(n)), "could not match the module's cuboids to the URDF's boxes"
    return [box_geom[j] for j in order]


def load_model():
    urdf = yourdfpy.URDF.load(URDF, load_meshes=True, build_scene_graph=True)
    joint_names = list(module.joint_names())
    assert [j.name for j in urdf.actuated_joints] == joint_names, "URDF and VAMP joint orders differ"
    assert list(module.end_effectors()) == EE_NAMES
    sphere_geom, box_geom = [], []
    for link in ET.parse(URDF).getroot().findall("link"):
        for col in link.findall("collision"):
            org, geom = col.find("origin"), col.find("geometry")
            xyz = np.array([float(x) for x in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
            rpy = [float(x) for x in (org.get("rpy", "0 0 0") if org is not None else "0 0 0").split()]
            if geom.find("box") is not None:
                box_geom.append((link.get("name"), xyz, rpy_matrix(*rpy), np.array([float(x) for x in geom.find("box").get("size").split()])))
            elif geom.find("sphere") is not None:
                sphere_geom.append((link.get("name"), xyz, float(geom.find("sphere").get("radius"))))
    box_geom = order_cuboids_like_vamp(box_geom, urdf)
    names = [g[0].split("/")[-1] for g in sphere_geom] + [g[0].split("/")[-1] + "[box]" for g in box_geom]
    print(f"{len(sphere_geom)} spheres and {len(box_geom)} boxes in the URDF; vamp has {module.n_spheres()} spheres, "
          f"{module.n_cuboids()} cuboids")
    lo, hi = np.asarray(module.lower_bounds(), float), np.asarray(module.upper_bounds(), float)
    q_lo, q_hi = lo.copy(), hi.copy()
    for i, n in enumerate(joint_names):
        if not (np.isfinite(lo[i]) and np.isfinite(hi[i])):
            q_lo[i], q_hi[i] = BASE_RANGES.get(n, (-np.pi, np.pi))
    return Model(urdf, joint_names, q_lo, q_hi, lo, hi, sphere_geom, box_geom, names)


# =====================================================================================================
# PLANNING SETUP: start configuration, curves, CoM constraint
# =====================================================================================================
@dataclass
class Setup:
    problem: Problem
    model: Model
    q_start: np.ndarray    # q0 projected onto the curve's first pose (VAMP's start, as the method does)
    plan_pos: np.ndarray   # the reference, its first pose moved to the tool poses at q0
    plan_quat: np.ndarray
    kp: np.ndarray         # padded keyframes (the kernel is traced for lerp_points() of them)
    kq: np.ndarray
    taus: np.ndarray       # shared t of the keyframes
    curves: list           # [the joint (12-row) curve constraint]
    com: object            # CoM constraint


def make_settings(method=PROJ_METHOD, tol=TOL, iterations=None):
    st = vamp.ConstraintSettings()
    st.method, st.tolerance, st.max_iterations = getattr(vamp.ProjMethod, method), tol, iterations or P.proj_iterations
    return st


def make_setup(problem, model):
    # the curve starts at the tools' actual poses at q0 (the benchmark's start_at_actual)
    model.urdf.update_cfg(problem.q0)
    plan_pos, plan_quat = problem.pos.copy(), problem.quat.copy()
    for e in range(2):
        T = model.urdf.get_transform(EE_NAMES[e], "world")
        plan_pos[0, e], plan_quat[0, e] = T[:3, 3], matrix_to_wxyz(T[:3, :3])
        if np.dot(plan_quat[0, e], problem.quat[1, e]) < 0:
            plan_quat[0, e] = -plan_quat[0, e]
        print(f"{EE_NAMES[e]}: curve start moved by {np.linalg.norm(plan_pos[0, e] - problem.pos[0, e]) * 1e3:.2f} mm from waypoint 0")
    n_points = int(module.lerp_points())
    kp, kq, _ = pad_keyframes_multi(plan_pos, plan_quat, n_points, ROT_WEIGHT)
    taus = np.asarray(shared_parameters(kp, kq, ROT_WEIGHT), float)
    curve = module.LerpSO3R3MultiConstraint(kp.transpose(1, 0, 2).tolist(), kq.transpose(1, 0, 2).tolist(), ROT_WEIGHT, False)
    curve.set_parameters([float(v) for v in taus])
    com = module.CoMConstraint(support_polygon(STABILITY_INSET, P.stability_margin).tolist())
    setup = Setup(problem, model, None, plan_pos, plan_quat, kp, kq, taus, [curve], com)
    start_settings = vamp.ConstraintSettings()
    start_settings.tolerance, start_settings.max_iterations = P.start_tolerance, P.start_iterations
    q_start = np.array(module.project(problem.q0.astype(np.float32), [region_at(setup, 0.0)], start_settings))
    print(f"start moved by {np.abs(q_start - problem.q0).max():.1e} rad (max over the {len(q_start)} joints)")
    setup.q_start = q_start
    return setup


def pose_at(setup, t):
    """[[qw qx qy qz x y z]] per end effector at curve parameter t."""
    i = int(np.clip(np.searchsorted(setup.taus, t, side="right") - 1, 0, len(setup.taus) - 2))
    u = (t - setup.taus[i]) / (setup.taus[i + 1] - setup.taus[i])
    return [[float(v) for v in (*tf.slerp(setup.kq[i, e], setup.kq[i + 1, e], u),
                                *(setup.kp[i, e] + u * (setup.kp[i + 1, e] - setup.kp[i, e])))] for e in range(2)]


def region_at(setup, t):
    return module.TaskSpaceConstraint([[1, 0, 0, 0, 0, 0, 0]] * 2, pose_at(setup, t), [[0.0] * 6] * 2, [[0.0] * 6] * 2)


def pair_names(setup, q):
    """The colliding pairs of q in VAMP's model, by link name."""
    names = setup.model.names
    return sorted({" <-> ".join(sorted((names[a], names[b]))) for a, b in module.debug(np.asarray(q, np.float32), env)[1]})


def report_start(setup):
    q = setup.q_start.astype(np.float32)
    print(f"VAMP validate(q_start): {module.validate(q, env, True)}  (without the bounds check: {module.validate(q, env, False)})")
    pairs = pair_names(setup, q)
    if pairs:
        print("  colliding pairs:", pairs)


# =====================================================================================================
# STEP 2: TSR CHAIN (what vamp_tsr_ik does)
# =====================================================================================================
def tsr_chain(setup):
    """Project the previous configuration onto each waypoint in turn; stop at the first failure or invalid result."""
    print("\nTSR chain (one projection per waypoint, both hands): waypoint, converged, valid, largest joint change")
    q_prev, pos = setup.q_start.astype(np.float32), setup.plan_pos
    for k in range(1, len(pos)):
        poses = [[float(v) for v in (*setup.plan_quat[k, e], *pos[k, e])] for e in range(2)]
        region = module.TaskSpaceConstraint([[1, 0, 0, 0, 0, 0, 0]] * 2, poses, [[0.0] * 6] * 2, [[0.0] * 6] * 2)
        try:
            q_k = np.asarray(module.project(q_prev, [region], make_settings()), np.float32)
        except ValueError as e:
            print(f"  k={k:<3} no convergence ({e}); chain stops")
            return k - 1
        valid = module.validate(q_k, env, True)
        d = np.abs(q_k - q_prev)
        print(f"  k={k:<3} converged=True  valid={valid!s:<5} max|dq|={d.max():.3f} ({setup.model.joint_names[int(d.argmax())].split('/')[-1]})")
        if not valid:
            print("        collides:", pair_names(setup, q_k))
            return k - 1
        q_prev = q_k
    return len(pos) - 1


# =====================================================================================================
# STEP 3: THE PLANNER
# =====================================================================================================
def plan_with(setup, parameter_weight, seed=None, stability=STABILITY, range_=RANGE, q_start=None, start_t=0.0):
    """One planner run from q_start (default: the problem's start) at curve parameter start_t."""
    pl = vamp.TaskSpaceRRTCSettings()
    pl.range, pl.max_greedy_steps = range_, MAX_GREEDY_STEPS
    pl.max_iterations, pl.max_samples = MAX_ITERATIONS, MAX_SAMPLES
    pl.project_samples, pl.parameter_weight = PROJECT_SAMPLES, parameter_weight
    pl.monotone_t = MONOTONE_T
    pl.walk_by_parameter = WALK_BY_PARAMETER
    if BASE_METRIC_WEIGHT:
        lo_b, hi_b = np.asarray(module.lower_bounds(), float), np.asarray(module.upper_bounds(), float)
        w_b = np.ones(len(lo_b)); w_b[:2] = np.max((hi_b - lo_b)[3:]) / (hi_b[0] - lo_b[0])
        pl.joint_weights = [float(v) for v in w_b]
    sampler = module.halton() if seed is None else module.xorshift(seed)
    cs = setup.curves + ([setup.com] if stability else [])
    q = setup.q_start if q_start is None else q_start
    return module.task_space_rrtc_curve(np.asarray(q, np.float32), env, pl, sampler, cs, make_settings(), start_t=float(start_t))


def run_planner(setup):
    """N_TRIALS planner runs per parameter weight; returns the first solved result (else the first one)."""
    print(f"\nplanner: {PROJ_METHOD}, range={RANGE}, max_iterations={MAX_ITERATIONS}, max_samples={MAX_SAMPLES}, "
          f"max_greedy_steps={MAX_GREEDY_STEPS}, project_samples={PROJECT_SAMPLES}, monotone_t={MONOTONE_T}, stability={STABILITY}")
    results = []
    for w in PARAMETER_WEIGHTS:
        runs = [plan_with(setup, w) for _ in range(N_TRIALS)]
        print(f"  parameter_weight={w:g}: {sum(r.solved for r in runs)}/{N_TRIALS} solved; iterations {[int(r.iterations) for r in runs]}")
        results += runs
    return next((r for r in results if r.solved), results[0])


def plan_path(setup, result):
    """The solved plan as dense joint knots (refined onto the curve like the method does), or None."""
    if not result.solved:
        return None
    states = np.array([np.asarray(q, float) for q in result.path])
    ts = np.asarray(list(result.t), float)
    q_path = np.vstack([setup.problem.q0, states])
    dense = module.interpolate_curve_path([np.asarray(q, np.float32) for q in result.path], [float(t) for t in ts], env,
                                          setup.curves + ([setup.com] if STABILITY else []), make_settings(), MAX_KNOT_STEP)
    print(f"interpolate_curve_path: interpolated={dense.interpolated}")
    if dense.interpolated:
        q_path = np.vstack([setup.problem.q0, np.array([np.asarray(q, float) for q in dense.path])])
    print(f"{len(states)} planner states; showing {len(q_path)} knots")
    return q_path


# =====================================================================================================
# STEP 4: THE WALK
# =====================================================================================================
def walk_project(setup, seed, t, method):
    st = make_settings(method, tol=P.start_tolerance, iterations=P.start_iterations)
    cs = [region_at(setup, t)] + ([setup.com] if STABILITY else [])
    try:
        return np.asarray(module.project(np.clip(seed, setup.model.lo, setup.model.hi).astype(np.float32), cs, st), float)
    except ValueError:
        return None


def classify(setup, q_from, t, o, t2, method):
    """Why candidate o (a projection onto pose(t2)) is rejected, or None if the walk can take it."""
    if o is None:
        return "no convergence"
    of = o.astype(np.float32)
    if not module.validate(of, env, True):
        return "joint limit" if module.validate(of, env, False) else "self-collision"   # validate(.., False) skips the bounds check
    d = module.interpolate_curve_path([q_from.astype(np.float32), of], [float(t), float(t2)], env,
                                      setup.curves + ([setup.com] if STABILITY else []), make_settings(method), MAX_KNOT_STEP)
    return None if d.interpolated else "edge check"


def walk_reference(setup, method, seed=1):
    """Returns the path [(t, q)] and, if it stalled, the candidates rejected at the stall [(reason, q or None)]."""
    rg, q, t, path = np.random.default_rng(seed), setup.q_start.copy(), 0.0, [(0.0, setup.q_start.copy())]
    while t < 1 - 1e-9:
        t2, rejected = min(1.0, t + WALK_STEP_T), []
        for k in range(WALK_CANDIDATES):
            seed_q = q if k == 0 else q + (0.03 if k < 20 else 0.1 if k < 40 else 0.3) * rg.standard_normal(len(q))
            o = walk_project(setup, seed_q, t2, method)
            why = classify(setup, q, t, o, t2, method)
            if why is None:
                q, t = o, t2
                path.append((t, q))
                break
            rejected.append((why, o))
        else:
            return path, rejected
    return path, []


def run_walks(setup):
    print(f"\nwalk (steps of {WALK_STEP_T} in t, {WALK_CANDIDATES} seeds per step, validity + edge check"
          f"{' + CoM' if STABILITY else ''}):")
    walks = {}
    for method in WALK_METHODS:
        path, rejected = walk_reference(setup, method)
        walks[method] = (path, rejected)
        print(f"  {method:<9}: reaches t={path[-1][0]:.2f}" + (f", STALLS; {len(rejected)} candidates rejected at the next step: "
              f"{dict(collections.Counter(w for w, _ in rejected))}" if rejected else " (end of the curve)"))
    return walks


# =====================================================================================================
# STEP 5: THE BASELINE (QP) PATH IN VAMP'S MODEL
# =====================================================================================================
def baseline_path(setup, method_name="diffik_joint_centering"):
    """The baseline's knots for this window in the planner's frame, and whether each is valid in VAMP's model."""
    from diffik_benchmark.backend import DrakeBackend
    from diffik_benchmark.methods import make_method
    from diffik_benchmark.robots import get_robot
    from diffik_benchmark.scoring import ScorerParams, score_segment

    pb = setup.problem
    spec, sp = get_robot("rby1"), ScorerParams()
    backend = DrakeBackend(spec)
    seg = Episode.load(f"{DATA}/trajectories/{DATASET}/ep_{pb.episode:05d}.npz").segment(pb.start_index, pb.n, pb.stride).transformed(
        pb.X_W_ref, spec.tool_ref_poses()).with_tolerance(sp.eps_pos, sp.eps_rot)
    out = make_method(method_name, {})(backend, pb.q0_bench, seg)
    success = score_segment(backend, seg, out, sp).success
    K = np.asarray(out.trajectory.knots, float)[:, pb.to_vamp]
    K[:, :2] -= pb.base_shift[:2]
    valid = np.array([module.validate(k.astype(np.float32), env, True) for k in K])
    valid_nb = np.array([module.validate(k.astype(np.float32), env, False) for k in K])
    print(f"\nbaseline {method_name}: scorer success={success}, {len(K)} knots; valid in VAMP's model: {int(valid.sum())}/{len(K)} "
          f"(ignoring joint bounds: {int(valid_nb.sum())}/{len(K)})")
    if not valid.all():
        first = int(np.argmin(valid))
        print(f"  first invalid knot {first}: colliding pairs {pair_names(setup, K[first]) or 'none (joint bounds)'}")
        oob = np.flatnonzero((K[first] < setup.model.lo) | (K[first] > setup.model.hi))
        print("  joints outside VAMP's bounds there:", [(setup.model.joint_names[j], round(float(K[first][j]), 4)) for j in oob] or "none")
    return K, valid


# =====================================================================================================
# STEP 6: VIEWER
# =====================================================================================================
class Viewer:
    """Viser scene: the robot's collision primitives (red when colliding), the reference, joint sliders, and
    any number of slider-driven sequences of configurations."""

    def __init__(self, setup):
        self.setup, self.model = setup, setup.model
        self.server = viser.ViserServer()
        self.server.scene.add_grid("/grid", width=4.0, height=4.0, plane="xy")
        self.server.scene.add_frame("/world", axes_length=0.1, axes_radius=0.003)
        self.robot = ViserUrdf(self.server, self.model.urdf, load_meshes=True, load_collision_meshes=False, root_node_name="/robot")
        self._add_reference()
        self._add_primitives()
        self._add_joint_sliders()

    def _add_reference(self):
        pb, setup = self.setup.problem, self.setup
        for e in range(2):
            self.server.scene.add_line_segments(f"/path/{EE_NAMES[e]}", points=np.stack([pb.pos[:-1, e], pb.pos[1:, e]], axis=1),
                                                colors=EE_COLORS[e], line_width=2.0)
            for i in range(len(pb.pos)):
                self.server.scene.add_frame(f"/waypoints/{EE_NAMES[e]}/{i:02d}", wxyz=pb.quat[i, e], position=pb.pos[i, e],
                                            axes_length=0.04, axes_radius=0.002, origin_radius=0.004)
                self.server.scene.add_label(f"/waypoints/{EE_NAMES[e]}/{i:02d}/label", text=str(i))
            self.server.scene.add_frame(f"/start_actual/{EE_NAMES[e]}", wxyz=setup.plan_quat[0, e], position=setup.plan_pos[0, e],
                                        axes_length=0.06, axes_radius=0.003, origin_radius=0.006)

    def _add_primitives(self):
        self.prims = []  # spheres, then boxes
        for i, (_, _, rad) in enumerate(self.model.sphere_geom):
            self.prims.append(self.server.scene.add_icosphere(f"/collision/sphere_{i}", radius=rad, color=GREY, opacity=0.3,
                                                              position=(0.0, 0.0, 0.0)))
        for i, (_, _, _, size) in enumerate(self.model.box_geom):
            self.prims.append(self.server.scene.add_box(f"/collision/box_{i}", dimensions=tuple(size), color=GREY, opacity=0.3,
                                                        position=(0.0, 0.0, 0.0)))
        with self.server.gui.add_folder("Collisions"):
            self.collision_text = self.server.gui.add_text("VAMP self-collision", "", disabled=True)
            show = self.server.gui.add_checkbox("collision primitives", True)

        @show.on_update
        def _(_):
            for h in self.prims:
                h.visible = show.value

    def _add_joint_sliders(self):
        m = self.model
        with self.server.gui.add_folder("Joints"):
            self.sliders = [self.server.gui.add_slider(n, min=float(m.q_lo[i]), max=float(m.q_hi[i]), step=1e-3,
                                                       initial_value=float(np.clip(self.setup.problem.q0[i], m.q_lo[i], m.q_hi[i])))
                            for i, n in enumerate(m.joint_names)]
            reset = self.server.gui.add_button("Reset to q_start")
        for s in self.sliders:
            s.on_update(lambda _: self.draw(np.array([s.value for s in self.sliders])))

        @reset.on_click
        def _(_):
            self.set_q(self.setup.q_start)

    def draw(self, q):
        """Robot and collision primitives at q (no slider update)."""
        m = self.model
        self.robot.update_cfg(q)
        m.urdf.update_cfg(np.asarray(q, float))
        hit = {i for pr in module.debug(np.asarray(q, np.float32), env)[1] for i in pr}
        n_s = len(m.sphere_geom)
        for i, h in enumerate(self.prims):
            link, off, *rest = m.sphere_geom[i] if i < n_s else m.box_geom[i - n_s]
            T = m.urdf.get_transform(link, "world")
            h.position = tuple(T[:3, :3] @ off + T[:3, 3])
            if i >= n_s:
                h.wxyz = matrix_to_wxyz(T[:3, :3] @ rest[0])
            h.color = RED if i in hit else GREY
            h.opacity = 0.6 if i in hit else 0.3
        self.collision_text.value = "; ".join(pair_names(self.setup, q)) or "none"

    def set_q(self, q):
        """Move the joint sliders (and so the robot) to q."""
        for s, v in zip(self.sliders, q):
            s.value = float(np.clip(v, s.min, s.max))
        self.draw(np.asarray(q, float))

    def add_sequence(self, folder, qs, labels):
        """A slider over configurations qs; labels[i] is shown for frame i."""
        with self.server.gui.add_folder(folder):
            slider = self.server.gui.add_slider("frame", min=0, max=max(len(qs) - 1, 1), step=1, initial_value=0)
            text = self.server.gui.add_text("info", labels[0], disabled=True)

        @slider.on_update
        def _(_):
            i = int(slider.value)
            self.set_q(qs[i])
            text.value = labels[i]

    def add_plots(self, path):
        """Two plots of what the reference asks for: how fast the joints must move along the exact walk (rad per unit t, the total and the six joints
        that move most), and the distance between consecutive reference waypoints for each hand."""
        setup, names = self.setup, self.model.joint_names
        t = np.array([p[0] for p in path]); Q = np.array([p[1] for p in path])
        dq = np.abs(np.diff(Q, axis=0)); dt = np.diff(t)[:, None]
        speed = dq / dt                                           # rad (m) per unit t, per joint
        top = np.argsort(-np.sum(dq, axis=0))[:6]
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
        short = lambda n: n.replace("left_arm", "L").replace("right_arm", "R").replace("torso", "T").replace("base/", "")
        with self.server.gui.add_folder("What the reference asks for"):
            self.server.gui.add_uplot(
                data=(t[1:], np.linalg.norm(speed, axis=1), *[speed[:, j] for j in top]),
                series=(uplot.Series(label="t"), uplot.Series(label="all joints (norm)", stroke="#000000", width=2),
                        *[uplot.Series(label=short(names[j]), stroke=c) for j, c in zip(top, colors)]),
                title="joint speed along the exact walk (rad or m per unit t)", aspect=1.6)
            P = setup.problem.pos
            step = np.linalg.norm(np.diff(P, axis=0), axis=2)     # (N, 2): metres between consecutive waypoints
            self.server.gui.add_uplot(
                data=(np.arange(1, len(P)), step[:, 0], step[:, 1]),
                series=(uplot.Series(label="waypoint"), uplot.Series(label="left hand", stroke=f"rgb{EE_COLORS[0]}"),
                        uplot.Series(label="right hand", stroke=f"rgb{EE_COLORS[1]}")),
                title="distance between consecutive reference waypoints (m)", aspect=1.6)

    def add_trace(self, name, qs, color_per_ee=EE_COLORS):
        """The end effectors' path through the configurations qs, as lines."""
        for e in range(2):
            trace = []
            for q in qs:
                self.model.urdf.update_cfg(np.asarray(q, float))
                trace.append(self.model.urdf.get_transform(EE_NAMES[e], "world")[:3, 3])
            trace = np.array(trace)
            self.server.scene.add_line_segments(f"/{name}/{EE_NAMES[e]}", points=np.stack([trace[:-1], trace[1:]], axis=1),
                                                colors=color_per_ee[e], line_width=3.0)


def run_viewer(setup, q_plan, walks, baseline):
    viewer = Viewer(setup)
    viewer.set_q(setup.q_start)
    if q_plan is not None:
        viewer.add_trace("planned", q_plan)
        viewer.add_sequence("Plan", q_plan, [f"plan knot {i} of {len(q_plan)}" for i in range(len(q_plan))])
    if walks:
        path, rejected = max(walks.values(), key=lambda w: w[0][-1][0])
        frames = [(f"walk t={t:.2f}", q) for t, q in path] + [(f"rejected at the next step ({w})", o) for w, o in rejected if o is not None]
        viewer.add_sequence("Walk", [q for _, q in frames], [label for label, _ in frames])
        viewer.add_plots(path)
    if baseline is not None:
        K, valid = baseline
        idx = np.linspace(0, len(K) - 1, min(len(K), 600)).astype(int)
        viewer.add_sequence("Baseline (QP) path", K[idx], [f"knot {i} of {len(K)}: " + ("valid in VAMP" if valid[i] else "INVALID in VAMP's model") for i in idx])
    print("\nviser running, Ctrl-C to quit")
    while True:
        time.sleep(1.0)


# =====================================================================================================
# MAIN
# =====================================================================================================
def describe_reference(setup, walks):
    """What the reference asks for, in words: the hands' travel and the joints' total motion along the exact walk."""
    P = setup.problem.pos
    travel = [float(np.sum(np.linalg.norm(np.diff(P[:, e], axis=0), axis=1))) for e in range(2)]
    path = max(walks.values(), key=lambda w: w[0][-1][0])[0]
    Q = np.array([q for _, q in path]); total = np.sum(np.abs(np.diff(Q, axis=0)), axis=0)
    names = setup.model.joint_names
    print(f"\nreference: hand travel over the window {travel[0]:.2f} m (left), {travel[1]:.2f} m (right); {len(P) - 1} waypoint steps")
    print(f"exact walk: total joint motion {total.sum():.1f} (sum of |dq| over joints, rad or m), {total.sum() / (path[-1][0] - path[0][0]):.0f} per unit t; "
          "the joints that move most (total, start -> end):")
    for j in np.argsort(-total)[:4]:
        print(f"   {names[j]:<26} {total[j]:6.2f}   ({Q[0][j]:+.2f} -> {Q[-1][j]:+.2f})")


def main():
    # python3 scripts/debug_rby1.py [start id]: any start of the set, N = 31, stride 6 (default: the problem at PROBLEM_INDEX)
    problem = load_problem(int(sys.argv[1]), 31, 6) if len(sys.argv) > 1 and sys.argv[1].isdigit() else load_problem(*PROBLEMS[PROBLEM_INDEX])
    model = load_model()
    setup = make_setup(problem, model)
    report_start(setup)
    if RUN_TSR_CHAIN:
        tsr_chain(setup)
    result = run_planner(setup) if RUN_PLANNER else None
    q_plan = plan_path(setup, result) if result is not None else None
    walks = run_walks(setup) if RUN_WALK else None
    if walks:
        describe_reference(setup, walks)
    baseline = baseline_path(setup) if RUN_BASELINE else None
    if RUN_VIEWER:
        run_viewer(setup, q_plan, walks, baseline)


if __name__ == "__main__":
    main()
