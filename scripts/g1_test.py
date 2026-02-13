import numpy as np
import os
from viser_utils import setup_viser_with_robot, add_spheres, add_trajectory
from pathlib import Path
import itertools
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
        [1.,     0.0005, 0.0005, 0.0005, 0.0, -0.303, 0.0],
        [-0.001, -0.001, -0.001, -0.001, -0.001, -0.001],
        [0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        [0.08, -0.045, 0.08, 0.045, 0.06, 0.045, 0.06, -0.045],
    )


    feet_tsr_constraint = vamp_module.TaskSpaceConstraint(
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.12, 0.11, -0.0], [1.0, 0.0, 0.0, 0.0, 0.12, -0.11, -0.0]],
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05],
        [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05]
    )


    constraints = vamp_module.Composable_T_C_B(feet_tsr_constraint, com_constraint, bimanual_constraint)



    e = vamp.Environment()
    problem_cuboids = np.loadtxt('resources/environments/cuboids/humanoid_shelf.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    # goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]


    goal = [0.03977,-0.00000,0.72049,0.00000,0.03753,-0.00002,-0.31120,0.00002,0.00004,0.64394,-0.41457,-0.00000,-0.31121,-0.00004,0.00000,0.64394,-0.41456,-0.00000,-0.00002,-0.00000,-0.24500,-0.00031,-0.00532,-0.00196,-0.00210,0.00362,-0.00156,0.00867,-0.00646,0.00363,-0.00701,0.00148,-0.00521,0.00134,0.01348]

    start = [-0.01691,-0.00008,0.52435,-0.00002,0.00087,0.00058,-0.89966,-0.00864,0.01527,1.75038,-0.87267,0.01757,-0.89936,0.00875,-0.01522,1.75040,-0.87267,-0.01753,1.59564,0.14700,0.37000,0.00002,0.00014,0.00009,-0.00003,-0.00006,0.00000,-0.00001,-0.00001,0.00017,0.00007,-0.00000,0.00003,0.00000,0.00002]

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
    ranges = [0.2, 0.5, 0.75, 1.0, 1.5]
    dyndoms = [False, True]
    all_combinations = list(itertools.product(ranges, dyndoms))
    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(1):
        for combination in all_combinations:
            plan_settings.range = combination[0]
            plan_settings.dynamic_domain = combination[1]
            result = planner_func(start, goal, e, plan_settings, constraints, sampler)

            planning_times[combination].append(result.nanoseconds/1e6)
    print("Execution completed")
    print("Planning times for each combination:")
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")


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
