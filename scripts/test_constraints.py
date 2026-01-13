import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path

import vamp
from fire import Fire
import time

problem = [
    # [0.55, 0, 0.25],
    # [0.55, 0, 0.50],
    # [0.55, 0, 0.60],
    [0.56, 0, 0.450],
    # [0.1, 0, 0.7],
    # # [0.35, 0.35, 0.25],
    # [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    # [0, -0.55, 0.25],
    # # [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
]
def main(
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,

    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("panda", "crrtc", **kwargs)
    )
    # tsr = vamp_module.TaskSpaceConstraint(
    #     [[0, 1,0,0,   0.3486, 0.647752, 0.2399]],
    #     [[1, 0, 0, 0, 0, 0, 0]],
    #     [-0.01, -10.01, -0.01, -0.01, -0.01, -0.01],
    #     [0.01, 10.01, 0.01, 0.01, 0.01, 0.01]
    # )
    tsr = vamp_module.TaskSpaceConstraint(
        [[1, 0, 0, 0, 0, 0, 0]],
        [[0, 0.707107, 0, 0.707107, 0.354, 0.7, 0.243]],
        [-10.01, -10.01, -0.01, -0.01, -0.01, -0.01],
        [10.01, 10.01, 0.01, 0.01, 0.01, 0.01]
    )

    # tf = np.identity(4)
    # tf[:3, 3] = np.array([0, 0, 0.02])

    # attachment = vamp.Attachment(tf)
    # attachment.add_spheres(
    #     [
    #         vamp.Sphere([0, 0, 0], 0.05),
    #         vamp.Sphere([0.1, 0, 0.0], 0.05),
    #         vamp.Sphere([-0.1, 0, 0.0], 0.05),
    #         # vamp.Sphere([0.0, 0, 0.3], 0.05),
    #     ]
    # )

    constraints = vamp_module.Composable_TaskSpaceConstraint(tsr)
    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))
    # e.add_cuboid(vamp.Cuboid([0, 0, -0.035], [0, 0, 0], [10, 10, 0.001]))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    start = [-1.053, -1.39, 1.878, -1.434, -0.531, 2.386, 2.761]
    goal = [-2.132, 1.558, 1.406, -1.452, 0.228, 2.444, -1.034]
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    np.set_printoptions(precision = 4, suppress = True)
    print(np.array(vamp_module.eefk(start)))
    print(np.array(vamp_module.eefk(goal)))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "panda_spherized.urdf")
    robot.update_cfg(start)

    floor_grid = server.scene.add_grid(
        name="/floor_grid",
        width=10.0,
        height=10.0,
        plane="xy",
        position=(0.0, 0.0, 0.0),
        cell_color=(200, 200, 200),
        section_color=(140, 140, 140),
    )


    leaf = server.scene.add_frame(
        "/tree/branch/leaf",
        wxyz=(0, 0.707107, 0, 0.707107),
        position=(0.354, 0.7, 0.243),

    )
    leaf = server.scene.add_frame(
        "/tree/branch/origin",
        wxyz=(1, 0, 0, 0),
        position=(0, 0, 0),

    )

    for _ in range(20):
        t1 = time.perf_counter_ns()
        result = planner_func(start, goal, e, plan_settings, constraints, sampler)
        print((time.perf_counter_ns() - t1) / 1e6)
    add_trajectory(
        server, result.path.numpy(), robot, [], [[]]
    )
    while True:
        continue


if __name__ == '__main__':
    Fire(main)
