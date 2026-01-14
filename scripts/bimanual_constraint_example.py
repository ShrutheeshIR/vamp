import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path

import vamp
from fire import Fire
import time

def main(
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,

    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("bimanual_panda", "crrtc", **kwargs)
    )
    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [0.0000348, 0.3745484, 0.9272074, 0.0000018, 0.0, 0.0, 0.171814],
        [-0.02, -0.02, -0.02, -0.1, -0.1, -0.1],
        [0.02, 0.02, 0.02, 0.1, 0.1, 0.1]
    )

    constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(bimanual_constraint)



    e = vamp.Environment()
    problem_cuboids = np.loadtxt('resources/environments/cuboids/shelf_drake.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    start = [-1.3238,  1.358 ,  1.0783, -2.4974,  0.5572,  2.5477, -1.4485, 1.2848,  1.2911, -1.0714, -2.4884, -0.6705,  2.5082,  0.7243]
    goal = [-1.997 ,  0.385 ,  2.1832, -2.0013,  1.3083,  1.8498, -0.7243, 1.2835,  1.3097, -2.0683, -2.1051, -0.1333,  2.4786, -0.7243]
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    np.set_printoptions(precision = 4, suppress = True)
    print(np.array(vamp_module.eefk(start)))
    print(np.array(vamp_module.eefk(goal)))

    robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    server, robot = setup_viser_with_robot(robot_dir, "bipanda_spherized.urdf")
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

    times = []
    for _ in range(20):
        t1 = time.perf_counter_ns()
        result = planner_func(start, goal, e, plan_settings, constraints, sampler)
        print((time.perf_counter_ns() - t1) / 1e6)
        times.append((time.perf_counter_ns() - t1) / 1e6)
    add_trajectory(
        server, result.path.numpy(), robot, [], [[]]
    )
    print(f"All times : {times}")
    print(f"Average time: {np.mean(times):.2f} ms")
    print(f"Standard deviation: {np.std(times):.2f} ms")
    while True:
        continue


if __name__ == '__main__':
    Fire(main)
