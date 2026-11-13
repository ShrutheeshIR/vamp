"""Build the planner's problem for a start of the rby1 hard sets (first 32-pose window, N=31, stride 6) in the method's frame (base xy at the
origin, yaw 0, tools rotated accordingly), the way vamp_curve_rrtc does."""
import contextlib, io
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
_spec = get_robot("rby1"); _b = DrakeBackend(_spec); _sp = ScorerParams(); _tool_ref = _spec.tool_ref_poses()
d.WALK_BY_PARAMETER, d.BASE_METRIC_WEIGHT = True, True
d.MAX_KNOT_STEP = 0.01   # the walk's edge check, as the benchmark's max_knot_step
with contextlib.redirect_stdout(io.StringIO()):
    model = d.load_model()
_names = list(d.module.joint_names())
_to_vamp = np.array([[n.replace("/", "_") for n in _b.scoped_joint_names].index(n.replace("/", "_")) for n in _names])
_cache = {}


def _load(ds):
    if ds not in _cache:
        _cache[ds] = (StartSet.load(starts_path("rby1", ds, STARTS_DIR)), {int(e.index): e for e in iter_episodes(TRAJECTORIES_DIR / ds, include_flagged=True)})
    return _cache[ds]


def build_case(ds, sid, n=N):
    """The first window of start sid (n poses). Returns the setup, or None if the start cannot be projected."""
    st, eps = _load(ds)
    return build_case_at(ds, sid, np.asarray(st.q[sid], float), int(st.start_index[sid]), n)


def build_case_at(ds, sid, q_bench, s, n=N):
    """The window of n poses (stride 6) starting at frame s of start sid's episode, from configuration q_bench (benchmark joint order and frame)."""
    st, eps = _load(ds); ep = eps[int(st.episode_index[sid])]; q_bench = np.asarray(q_bench, float)
    seg = ep.segment(s, n, STRIDE).transformed(st.X_W_ref[sid], _tool_ref).with_tolerance(_sp.eps_pos, _sp.eps_rot)
    q0 = q_bench[_to_vamp].copy(); shift = np.zeros(23); shift[:2] = q0[:2]; q0 -= shift
    yaw0 = float(q0[2]); R = np.array([[np.cos(yaw0), -np.sin(yaw0)], [np.sin(yaw0), np.cos(yaw0)]]); qz = np.array([np.cos(-yaw0 / 2), 0, 0, np.sin(-yaw0 / 2)])
    q0[2] = 0.0
    pos = seg.pos[:, d.EE_ORDER] - np.array([shift[0], shift[1], 0.0]); pos = pos.copy(); pos[..., :2] = pos[..., :2] @ R
    quat = tf.quat_mul(qz, seg.quat[:, d.EE_ORDER])
    problem = d.Problem(sid, n, STRIDE, int(st.episode_index[sid]), s, st.X_W_ref[sid], q0, q_bench, shift, _to_vamp, pos, quat)
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            return d.make_setup(problem, model)
    except Exception:
        return None
