import numpy as np
from viser import transforms as tf
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path
import json
import vamp
from fire import Fire
from scipy.spatial.transform import Rotation as R
import itertools

np.set_printoptions(suppress = True)
# Starting configuration
a = [-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294]

# Goal configuration
b = [1.40490, 0.35201, -0.22762, -1.90963, 0.10796, 2.26183, 0.22238]

# Problem specification: a list of sphere centers
problem = [
    # [0.55, 0, 0.25],
    # [0.35, 0.35, 0.25],
    # [0, 0.55, 0.25],
    # [-0.55, 0, 0.25],
    # [-0.35, -0.35, 0.25],
    # [0, -0.55, 0.25],
    # [0.35, -0.35, 0.25],
    # [0.35, 0.35, 0.8],
    # [0, 0.55, 0.8],
    # [-0.35, 0.35, 0.8],
    # [-0.55, 0, 0.8],
    # [-0.35, -0.35, 0.8],
    # [0, -0.55, 0.8],
    # [0.35, -0.35, 0.8],
    ]

def add_json_cuboids(server, e, filename, color=(12, 89, 178)):
    """
    Add cuboids from a JSON file. The JSON should be a list of objects, each with keys:
    'name', 'x', 'y', 'z', 'dx', 'dy', 'dz' and optional 'roll','pitch','yaw'.

    This function reads the JSON, iterates over entries, and adds a box for each item.
    Dimensions are used as provided in the JSON (assumed full extents).
    """
    with open(filename, "r") as f:
        data = json.load(f)

    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x = obj.get("x", 0.0)
        y = obj.get("y", 0.0)
        z = obj.get("z", 0.0)
        dx = obj.get("dx", 0.0)
        dy = obj.get("dy", 0.0)
        dz = obj.get("dz", 0.0)

        e.add_cuboid(vamp.Cuboid([x, y, z], [0.0, 0.0, 0.0], [dx / 2, dy / 2, dz / 2]))

        # Position and dimensions expected as (x, y, z) and (dx, dy, dz)
        server.scene.add_box(
            name=f"/{name}",
            dimensions=(dx, dy, dz),
            color=color,
            position=(x, y, z),
        )


def main(
    obstacle_radius: float = 0.2,
    attachment_radius: float = 0.01,
    planner: str = "crrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    # Create an attachment offset on the Z-axis from the end-effector frame
    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, 0])
    attachment = vamp.Attachment(tf)

    attach_positions = np.zeros((10, 3))
    attach_positions[:, 2] = np.linspace(0, 0.18, len(attach_positions))
    print(attach_positions)

    # Add a single sphere to the attachment - spheres are added in the attachment's local frame
    attachment.add_spheres(
        [
            # vamp.Sphere([0, 0, 0], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.02], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.04], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.06], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.08], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.1], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.12], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.14], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.16], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.18], attachment_radius),
            # vamp.Sphere([0.0, 0, 0.2], attachment_radius),
            vamp.Sphere(pos, attachment_radius) for pos in attach_positions
        ]
    )

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot.update_cfg(a)

    fka = vamp_module.eefk(a)
    fkb = vamp_module.eefk(b)


    print(R.from_matrix(fka[0][:3, :3]).as_quat(scalar_first = True), fka[0][:3, 3])
    print(R.from_matrix(fkb[0][:3, :3]).as_quat(scalar_first = True), fkb[0][:3, 3])

    tsr = vamp_module.TaskSpaceConstraint(
        [[1, 0, 0, 0, 0, 0, 0]],
        [[0, 1.0, 0, 0.0, 0.29276255, -0.55347496, 0.20607783]],
        [-10.01, -10.01, -0.01, -0.01, -0.01, -10.01],
        [10.01, 10.01, 0.01, 0.01, 0.01, 10.01]
    )
    constraints = vamp_module.Composable_TaskSpaceConstraint(tsr)

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))

    _problem_sphere_handles = add_spheres(
        server, np.array(problem), np.array([obstacle_radius] * len(problem))
        )

    add_json_cuboids(server, e, "resources/environments/cuboids/real_maze.json")

    # Add the attchment to the VAMP environment
    e.attach(attachment, 0)
    # Add attachment sphere to visualization
    attachment_sph_groups = []
    attachment_sph_groups.append(
        add_spheres(
            server,
            attach_positions,
            [attachment_radius] * len(attach_positions),
            colors=[[0, 255, 0]] * len(attach_positions),
            prefix="attach1",
        )
    )

    # Update attachment sphere positions corresponding to the waypoints.
    # this could also be made into a callable that can be called during trajectory viz
    def get_attachment_pos(configuration):
        for idx, attach in enumerate([attachment]):
            attach.set_ee_pose(vamp_module.eefk(configuration)[idx])
        return [
            np.array([sph.position for sph in attachment.posed_spheres]),
        ]

    # Plan and display
    sampler = vamp_module.halton()
    plan_settings.range = 0.75
    # result = planner_func(a, b, e, plan_settings, sampler)
    result = planner_func(a, b, e, plan_settings, constraints, sampler)

    simple = vamp_module.simplify_with_constraints(result.path, e, constraints, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    attachment_positions = [get_attachment_pos(pos) for pos in simple.path.numpy()]

    add_trajectory(
        server, simple.path.numpy(), robot, attachment_sph_groups, attachment_positions
    )
    ranges = [0.5, 0.75]
    dyndoms = [False, True]
    all_combinations = list(itertools.product(ranges, dyndoms))
    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(1):
        for combination in all_combinations:
            plan_settings.range = combination[0]
            plan_settings.dynamic_domain = combination[1]
            result = planner_func(a, b, e, plan_settings, constraints, sampler)
            print(result.nanoseconds / 1e3)
            planning_times[combination].append(result.nanoseconds/1e6)

            attachment_positions = [get_attachment_pos(pos) for pos in result.path.numpy()]

            add_trajectory(
                server, result.path.numpy(), robot, attachment_sph_groups, attachment_positions
            )

    print("Execution completed")
    print("Planning times for each combination:")
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")


    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
