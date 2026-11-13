"""Which stage kills the planner's extensions? From a node (q, t) take random samples as explore() does: step toward a random (q, t) by
at most `range` in the metric, project onto the curve at the new t, then the checks of steer(): drift <= 2 * step, endpoint valid,
traced path (interpolate_curve_path). Counts the first failing stage. (Python copy of the steps; the planner's lanes and their
perturbations are not copied.)"""
import collections
import sys
sys.path.insert(0, "/src/vamp/scripts")
import numpy as np
import debug_rby1 as d

problem = d.load_problem(*d.PROBLEMS[d.PROBLEM_INDEX]); model = d.load_model(); setup = d.make_setup(problem, model)
path, _ = d.walk_reference(setup, "BoundedLS")
by_t = {round(t, 2): q for t, q in path}
W, R, TRIALS = d.PARAMETER_WEIGHTS[0], d.RANGE, 500
rng = np.random.default_rng(0)
cs_extra = [setup.com] if d.STABILITY else []
st = d.make_settings()


def extend(q, t):
    q_s, t_s = rng.uniform(model.q_lo, model.q_hi), rng.uniform(0, 1)
    t_s = max(t_s, t)                                    # monotone_t: a sample behind the node is steered toward at the node's t
    dist = np.sqrt(np.sum((q_s - q) ** 2) + (W * (t_s - t)) ** 2)
    k = min(1.0, R / dist)
    q1, t1 = q + (q_s - q) * k, t + (t_s - t) * k        # the step toward the sample
    step = min(dist, R)
    raw_ok = bool(d.module.validate(np.clip(q1, model.lo, model.hi).astype(np.float32), d.env, True))
    try:
        qp = np.asarray(d.module.project(np.clip(q1, model.lo, model.hi).astype(np.float32), [d.region_at(setup, t1)] + cs_extra, st), float)
    except ValueError:
        return "projection does not converge", raw_ok, None
    drift = np.sum((qp - q) ** 2)
    if drift > 4 * step ** 2:
        return "drift > 2 x step", raw_ok, np.sqrt(drift)
    if not d.module.validate(qp.astype(np.float32), d.env, True):
        return ("endpoint: self-collision" if d.module.validate(qp.astype(np.float32), d.env, False) else "endpoint: joint limit"), raw_ok, np.sqrt(drift)
    ok = d.module.interpolate_curve_path([q.astype(np.float32), qp.astype(np.float32)], [float(t), float(t1)], d.env,
                                         setup.curves + cs_extra, st, d.MAX_KNOT_STEP).interpolated
    return ("success" if ok else "traced path fails (continuity/collision)"), raw_ok, np.sqrt(drift)


print(f"\nrange={R}, parameter_weight={W:g}, {TRIALS} random samples per node; first failing stage of each extension")
for t0 in (0.0, 0.3, 0.7, 0.8):
    q = by_t[t0]
    res = [extend(q, t0) for _ in range(TRIALS)]
    c = collections.Counter(r[0] for r in res)
    raw = np.mean([r[1] for r in res])
    drifts = [r[2] for r in res if r[2] is not None]
    print(f"\nnode t={t0}: raw straight step valid in VAMP's model before projecting: {raw:.0%}; median |dq| after projection {np.median(drifts):.3f}")
    for k_, v in c.most_common():
        print(f"   {v / TRIALS:5.0%}  {k_}")
