"""Browse every start configuration of the RB-Y1 hard sets in viser and flag the
ones that collide in VAMP's collision model (spheres + cuboids) or lie outside its joint bounds.

A slider picks the start id; a dropdown picks the dataset and a filter
(all / colliding / valid) restricts the slider to those starts. The colliding
spheres and boxes are drawn in red and the colliding link pairs are listed.

Run in the diff-ik-benchmarks container (vamp built with VAMP_ROBOTS=rby1_tool):
    python3 -u scripts/debug_rby1_starts.py          # viser on :8080
    python3 -u scripts/debug_rby1_starts.py --check  # print the summary and exit
    python3 -u scripts/debug_rby1_starts.py --export starts.html   # one self-contained page to send around
    python3 -u scripts/debug_rby1_starts.py --export starts.viser [--export-filter all|colliding|valid|"out of bounds"] [--export-max N]
                                                     # offline recording: timeline + free camera, no sliders
"""
import json
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import viser
import yourdfpy
from viser.extras import ViserUrdf

import vamp

DATASETS = ["nymeria_L60x180_v01", "nymeria_x180fill_v01"]
DATA = "/src/diffik-benchmark/data"
# The model VAMP was generated from: its collision primitives (spheres first, then boxes,
# each in URDF order) are named and drawn here, and its visual meshes (relative to the
# URDF, cricket/resources/rby1_tool/meshes) are the solid robot.
COLLISION_URDF = "/src/cricket/resources/rby1_tool/rby1_primitive.urdf"
CHECK = "--check" in sys.argv

module = vamp.rby1_tool
env = vamp.Environment()


# --- collision geometry (to name and draw the colliding parts) -------------------


def order_cuboids_like_vamp(box_geom, urdf, module):
    """The URDF's boxes in the order of the module's cuboids (the generated header orders them by link, not
    as the URDF lists them): each cuboid is matched to the box whose world centre it has, at a few
    configurations."""
    n = len(box_geom)
    if n == 0 or not hasattr(module, "cuboid_fk"):
        return box_geom
    rng_ = np.random.default_rng(5)
    lo_, hi_ = np.asarray(module.lower_bounds(), float), np.asarray(module.upper_bounds(), float)
    qs_ = []
    for _ in range(4):
        q_ = np.clip(rng_.uniform(-0.8, 0.8, len(lo_)), lo_, hi_)
        q_[:3] = rng_.uniform(-1, 1, 3)
        qs_.append(q_)
    urdf_centers, vamp_centers = [], []
    for q_ in qs_:
        urdf.update_cfg(q_)
        urdf_centers.append(np.array([urdf.get_transform(link, "world")[:3, :3] @ off + urdf.get_transform(link, "world")[:3, 3]
                                      for link, off, _, _ in box_geom]))
        vamp_centers.append(np.array([[c.x, c.y, c.z] for c in module.cuboid_fk(q_.astype(np.float32))]))
    order = []
    for i in range(n):
        cost = [sum(np.linalg.norm(vamp_centers[k][i] - urdf_centers[k][j]) for k in range(len(qs_))) for j in range(n)]
        order.append(int(np.argmin(cost)))
    assert sorted(order) == list(range(n)), "could not match the module's cuboids to the URDF's boxes"
    return [box_geom[j] for j in order]

def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def rot_wxyz(Rw):
    w = np.sqrt(max(0.0, 1.0 + Rw[0, 0] + Rw[1, 1] + Rw[2, 2])) / 2
    if w > 1e-6:
        return (w, (Rw[2, 1] - Rw[1, 2]) / (4 * w), (Rw[0, 2] - Rw[2, 0]) / (4 * w),
                (Rw[1, 0] - Rw[0, 1]) / (4 * w))
    k = int(np.argmax(np.diag(Rw)))  # 180 degree rotation
    v = np.sqrt(max(0.0, 1 + 2 * Rw[k, k] - np.trace(Rw))) / 2
    a = np.zeros(3)
    a[k] = v
    for j in range(3):
        if j != k:
            a[j] = (Rw[j, k] + Rw[k, j]) / (4 * v)
    return (0.0, *a)


spheres, boxes = [], []  # (link, xyz, R, radius) / (link, xyz, R, size)
for link in ET.parse(COLLISION_URDF).getroot().findall("link"):
    for col in link.findall("collision"):
        org = col.find("origin")
        xyz = np.array([float(v) for v in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
        rpy = [float(v) for v in (org.get("rpy", "0 0 0") if org is not None else "0 0 0").split()]
        R = rpy_matrix(*rpy)
        g = col.find("geometry")
        if g.find("sphere") is not None:
            spheres.append((link.get("name"), xyz, R, float(g.find("sphere").get("radius"))))
        elif g.find("box") is not None:
            boxes.append((link.get("name"), xyz, R,
                          np.array([float(v) for v in g.find("box").get("size").split()])))
n_spheres = len(spheres)
geom_names = [s[0].split("/")[-1] for s in spheres] + [b[0].split("/")[-1] + "[box]" for b in boxes]
print(f"collision urdf: {n_spheres} spheres, {len(boxes)} boxes; "
      f"vamp: {module.n_spheres()} spheres, {getattr(module, 'n_cuboids', lambda: 0)()} cuboids")
assert n_spheres == module.n_spheres(), "COLLISION_URDF is not the model VAMP was built from"

# --- starts: q in VAMP's joint order, and whether each one collides ------------------
urdf = yourdfpy.URDF.load(COLLISION_URDF, load_meshes=True, build_scene_graph=True)
server = viser.ViserServer()
server.scene.add_grid("/grid", width=4.0, height=4.0, plane="xy")
server.scene.add_frame("/world", axes_length=0.1, axes_radius=0.003)
robot = ViserUrdf(server, urdf, load_meshes=True, load_collision_meshes=False,
                  root_node_name="/robot")
joint_names = robot.get_actuated_joint_names()
assert list(module.joint_names()) == list(joint_names), "URDF and VAMP joint orders differ"
# the module's cuboids are not in the URDF's order: reorder the boxes to match them
boxes = order_cuboids_like_vamp(boxes, urdf, module)
geom_names = [s_[0].split("/")[-1] for s_ in spheres] + [b_[0].split("/")[-1] + "[box]" for b_ in boxes]

sets = {}
for ds in DATASETS:
    z = np.load(f"{DATA}/starts/rby1/{ds}.npz")
    meta = json.load(open(f"{DATA}/starts/rby1/{ds}.json"))
    bench = [n.split("/")[-1] for n in meta["joint_names"]]
    to_vamp = np.array([bench.index(n.split("/")[-1]) for n in joint_names])
    Q = np.asarray(z["q"], float)[:, to_vamp]
    pairs = [module.debug(q.astype(np.float32), env)[1] for q in Q]
    lo = np.asarray(module.lower_bounds(), float)
    hi = np.asarray(module.upper_bounds(), float)
    oob = [np.flatnonzero((q < lo) | (q > hi)) for q in Q]
    coll = np.array([len(p) > 0 for p in pairs])
    bounds = np.array([len(o) > 0 for o in oob])
    valid = ~coll & ~bounds
    sets[ds] = dict(Q=Q, pairs=pairs, valid=valid, coll=coll, bounds=bounds, oob=oob, lo=lo, hi=hi,
                    episode=z["episode_index"], start=z["start_index"])
    print(f"{ds}: {valid.sum()} valid, {coll.sum()} colliding, {bounds.sum()} outside joint bounds "
          f"({(coll & bounds).sum()} both) of {len(Q)}")
    top = {}
    for p in pairs:
        for k in {tuple(sorted((geom_names[a], geom_names[b]))) for a, b in p}:
            top[k] = top.get(k, 0) + 1
    for k, c in sorted(top.items(), key=lambda kv: -kv[1])[:5]:
        print(f"    {c:5d}  {k[0]} <-> {k[1]}")
if CHECK:
    sys.exit(0)

# --- gui ---------------------------------------------------------------------------
with server.gui.add_folder("Start"):
    dataset = server.gui.add_dropdown("dataset", DATASETS)
    filt = server.gui.add_dropdown("show", ["all", "colliding", "out of bounds", "valid"])
    slider = server.gui.add_slider("start id", min=0, max=len(sets[DATASETS[0]]["Q"]) - 1, step=1,
                                   initial_value=0)
    prev_btn = server.gui.add_button("prev")
    next_btn = server.gui.add_button("next")
    status = server.gui.add_text("VAMP", "", disabled=True)
    pairs_text = server.gui.add_text("colliding pairs", "", disabled=True)
    oob_text = server.gui.add_text("joint bounds", "", disabled=True)
    info = server.gui.add_text("start", "", disabled=True)
    counts = server.gui.add_text("counts", "", disabled=True)

GREY, RED = (120, 150, 200), (230, 40, 40)
prims = []  # one persistent scene handle per collision primitive, spheres then boxes
for i, (link, off, R, rad) in enumerate(spheres):
    prims.append(server.scene.add_icosphere(f"/collision/sphere_{i}", radius=rad, color=GREY,
                                            opacity=0.35, position=(0.0, 0.0, 0.0)))
for i, (link, off, R, size) in enumerate(boxes):
    prims.append(server.scene.add_box(f"/collision/box_{i}", dimensions=tuple(size), color=GREY,
                                      opacity=0.35, position=(0.0, 0.0, 0.0)))
with server.gui.add_folder("Display"):
    show_visual = server.gui.add_checkbox("visual meshes", True)
    show_prims = server.gui.add_checkbox("collision primitives", True)
    prim_opacity = server.gui.add_slider("primitive opacity", min=0.05, max=1.0, step=0.05,
                                         initial_value=0.35)


def current_ids():
    s = sets[dataset.value]
    if filt.value == "colliding":
        return np.flatnonzero(s["coll"])
    if filt.value == "out of bounds":
        return np.flatnonzero(s["bounds"])
    if filt.value == "valid":
        return np.flatnonzero(s["valid"])
    return np.arange(len(s["Q"]))


ids = current_ids()


def show(sid, ds=None):
    ds = ds or dataset.value
    s = sets[ds]
    q = s["Q"][sid]
    # for h in shown:
    #     h.remove()
    # shown.clear()
    robot.update_cfg(q)
    urdf.update_cfg(q)
    hit = {i for p in s["pairs"][sid] for i in p}
    pr = s["pairs"][sid]
    for i, h in enumerate(prims):
        link, off, R, _ = spheres[i] if i < n_spheres else boxes[i - n_spheres]
        T = urdf.get_transform(link, "world")
        h.position = tuple(T[:3, :3] @ off + T[:3, 3])
        if i >= n_spheres:
            h.wxyz = rot_wxyz(T[:3, :3] @ R)
        h.color = RED if i in hit else GREY
        h.opacity = max(prim_opacity.value, 0.6) if i in hit else prim_opacity.value
    names = sorted({" <-> ".join(sorted((geom_names[a], geom_names[b]))) for a, b in pr})
    flags = []
    if s["coll"][sid]:
        flags.append(f"COLLISION ({len(names)} pairs)")
    if s["bounds"][sid]:
        flags.append("OUT OF JOINT BOUNDS")
    status.value = " + ".join(flags) or "valid"
    pairs_text.value = "; ".join(names) or "none"
    oob_text.value = "; ".join(
        f"{joint_names[j]} {s['Q'][sid][j]:.4f} not in [{s['lo'][j]:.3f}, {s['hi'][j]:.3f}]"
        for j in s["oob"][sid]) or "none"
    server.scene.add_label("/status", text=f"{ds} id {sid}: {status.value}", position=(0.0, 0.0, 2.1))
    info.value = f"id {sid}: episode {int(s['episode'][sid])}, start {int(s['start'][sid])}"


def refresh(_=None):
    global ids
    ids = current_ids()
    s = sets[dataset.value]
    counts.value = (f"{len(ids)} shown; {int(s['valid'].sum())} valid, {int(s['coll'].sum())} colliding, "
                    f"{int(s['bounds'].sum())} out of bounds of {len(s['Q'])}")
    if len(ids) == 0:
        status.value, pairs_text.value, oob_text.value = "no starts match", "", ""
        return
    slider.max = len(ids) - 1
    slider.value = min(int(slider.value), len(ids) - 1)
    show(int(ids[int(slider.value)]))


@show_prims.on_update
def _(_):
    for h in prims:
        h.visible = show_prims.value


@show_visual.on_update
def _(_):
    if hasattr(robot, "show_visual"):
        robot.show_visual = show_visual.value


@prim_opacity.on_update
def _(_):
    if len(ids):
        show(int(ids[int(slider.value)]))


@slider.on_update
def _(_):
    if len(ids):
        show(int(ids[int(slider.value)]))


@prev_btn.on_click
def _(_):
    slider.value = max(0, int(slider.value) - 1)


@next_btn.on_click
def _(_):
    slider.value = min(len(ids) - 1, int(slider.value) + 1)


def arg(name, default=None):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


EXPORT = arg("--export")
if EXPORT:
    # A .viser recording: one frame per start (DWELL s each), no GUI. Played back it has a
    # timeline and a free camera, but no joint sliders (GUI messages are not recorded).
    flt, cap, DWELL = arg("--export-filter", "all"), int(arg("--export-max", 10**9)), 0.25
    import os
    import pathlib
    ser = server.get_scene_serializer()
    todo = sum(len(np.flatnonzero({"all": np.ones(len(sets[d]["Q"]), bool), "colliding": sets[d]["coll"],
                                   "valid": sets[d]["valid"], "out of bounds": sets[d]["bounds"]}[flt]))
               if cap >= 10**9 else min(cap, int(len(sets[d]["Q"]))) for d in DATASETS)
    print(f"[export] recording {todo} starts (filter={flt}) to {os.path.abspath(EXPORT)}", flush=True)
    t_export = time.time()
    n = 0
    for ds in DATASETS:
        s = sets[ds]
        pick = {"all": np.ones(len(s["Q"]), bool), "colliding": s["coll"], "valid": s["valid"],
                "out of bounds": s["bounds"]}[flt]
        idx = np.flatnonzero(pick)
        if len(idx) > cap:
            idx = idx[np.linspace(0, len(idx) - 1, cap).astype(int)]
        for sid in idx:
            show(int(sid), ds)
            ser.insert_sleep(DWELL)
            n += 1
            if n % 50 == 0 or n == todo:
                print(f"[export] {n}/{todo} starts ({100 * n / max(todo, 1):.0f}%), "
                      f"{time.time() - t_export:.0f} s", flush=True)
    print("[export] serializing...", flush=True)
    path = pathlib.Path(EXPORT)
    # .html: one self-contained page (viewer + recording), opens by double click, no server.
    data = ser.as_html().encode() if path.suffix == ".html" else ser.serialize()
    print(f"[export] writing {len(data) / 1e6:.1f} MB...", flush=True)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    print(f"[export] DONE: wrote {path.resolve()} ({path.stat().st_size / 1e6:.1f} MB, {n} starts, "
          f"{n * DWELL:.0f} s of playback)", flush=True)
    sys.exit(0)

dataset.on_update(refresh)
filt.on_update(refresh)
refresh()
print("viser running (the slider indexes into the filtered list; the start field shows the real id), Ctrl-C to quit")
while True:
    time.sleep(1.0)
