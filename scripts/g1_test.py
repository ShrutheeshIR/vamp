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
        vamp.configure_robot_and_planner_with_kwargs("g1_unitree", "crrtc", **kwargs)
    )
    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [1.00, 0.0, 0.00, 0.00, 0.0, -0.303, 0.0],
        [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001],
        [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    )

    constraints = vamp_module.Composable_BimanualTaskSpaceConstraint(bimanual_constraint)



    e = vamp.Environment()
    # problem_cuboids = np.loadtxt('resources/environments/cuboids/shelf_drake.txt', delimiter = ",")
    # for cuboid in problem_cuboids:
    #     e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    start = [-0.01405,0.00000,-0.20000,0.00000,0.00060,0.00064,-0.89950,-0.00000,-0.00000,1.75013,-0.87266,-0.00000,-0.89950,0.00000,0.00000,1.75013,-0.87266,-0.00000,1.59564,-0.27786,0.51984,0.00002,0.00014,0.00009,-0.00003,-0.00006,-0.00000,-0.00001,-0.00001,0.00017,0.00007,-0.00000,0.00003,-0.00000,0.00002 ]
    goal = [0.00539,0.00009,0.03086,0.00000,-0.01316,0.00001,0.00027,0.00001,-0.00002,0.00900,0.01995,-0.00001,0.00025,0.00001,0.00001,0.00900,0.01998,-0.00001,0.00001,-0.00000,-0.00796,0.00159,0.00003,-0.00010,-0.00000,-0.00017,-0.00001,-0.00032,0.00148,-0.00011,-0.00008,0.00008,0.00009,0.00001,0.00013]
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    np.set_printoptions(precision = 4, suppress = True)
    print(np.array(vamp_module.eefk(start)))
    print(np.array(vamp_module.eefk(goal)))

    # robot_dir = Path(__file__).parents[1] / "resources" / "panda"
    # server, robot = setup_viser_with_robot(robot_dir, "bipanda_spherized.urdf")
    # robot.update_cfg(start)

    # floor_grid = server.scene.add_grid(
    #     name="/floor_grid",
    #     width=10.0,
    #     height=10.0,
    #     plane="xy",
    #     position=(0.0, 0.0, 0.0),
    #     cell_color=(200, 200, 200),
    #     section_color=(140, 140, 140),
    # )


    # leaf = server.scene.add_frame(
    #     "/tree/branch/leaf",
    #     wxyz=(0, 0.707107, 0, 0.707107),
    #     position=(0.354, 0.7, 0.243),

    # )
    # leaf = server.scene.add_frame(
    #     "/tree/branch/origin",
    #     wxyz=(1, 0, 0, 0),
    #     position=(0, 0, 0),

    # )

    # times = []
    # for _ in range(20):
    #     t1 = time.perf_counter_ns()
    #     result = planner_func(start, goal, e, plan_settings, constraints, sampler)
    #     print((time.perf_counter_ns() - t1) / 1e6)
    #     times.append((time.perf_counter_ns() - t1) / 1e6)
    # add_trajectory(
    #     server, result.path.numpy(), robot, [], [[]]
    # )
    # print(f"All times : {times}")
    # print(f"Average time: {np.mean(times):.2f} ms")
    # print(f"Standard deviation: {np.std(times):.2f} ms")
    # while True:
    #     continue


if __name__ == '__main__':
    Fire(main)
