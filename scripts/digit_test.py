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

def run_planner(
    combination,
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,
    **kwargs,
):

    start = [0.00000,0.00184,-0.00516,-0.00096,0.00227,-0.00049,0.36547,-0.00525,0.29127,0.31631,-0.28807,0.11635,-0.00983,0.02358,-0.00579,0.01520,0.09714,-0.36544,0.00508,-0.29158,-0.31758,0.28939,-0.11670,0.01291,0.00435,0.00820,-0.01199,-0.09980]
    goal = [0.00358,0.01604,-0.48348,-0.00927,0.00030,-0.00362,0.37249,0.00746,-0.22666,-0.85028,0.91164,-0.43089,0.01204,-0.17608,-0.40123,0.03686,1.07578,-0.34264,0.01123,0.22809,0.84257,-0.90335,0.43159,0.05288,-0.28604,0.42121,-0.16502,-1.13728]


    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc", **kwargs)
    )
    print("Created vamp module")
    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356],
        [-0.001, -0.001, -0.001, -10.1, -10.1, -10.1],
        [0.001, 0.001, 0.001, 10.1, 10.1, 10.1]
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        [0.03, -0.075, 0.03, 0.075, -0.04, 0.075, -0.04, -0.075],
    )


    feet_tsr_constraint = vamp_module.TaskSpaceConstraint(
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [0.603, 0.36, 0.36, 0.603, -0.04302,  0.10080, -0.96013], [0.603, -0.36, 0.36, -0.603 , -0.04288, -0.09895, -0.96033]],
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.001, -0.01, -0.01, -0.05, -0.05, -0.05, -0.01, -0.01, -0.01, -0.05, -0.05, -0.05],
        [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.01, 0.01, 0.001, 0.05, 0.05, 0.05, 0.01, 0.01, 0.01, 0.05, 0.05, 0.05]
    )


    constraints = vamp_module.Composable_T_C_B(feet_tsr_constraint, com_constraint, bimanual_constraint)



    e = vamp.Environment()
    problem_cuboids = np.loadtxt('resources/environments/cuboids/humanoid_digit_shelf.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6] / 2))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    # np.set_printoptions(precision = 4, suppress = True)
    # print(np.array(vamp_module.eefk(start)))
    # print(np.array(vamp_module.eefk(goal)))

    plan_settings.range = combination[0]
    plan_settings.dynamic_domain = combination[1]
    plan_settings.max_iterations = 10000
    print("Started planner")
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    print(result)

    return result


if __name__ == '__main__':
    ranges = [0.5, 0.75, 1.0, 1.5]
    dyndoms = [False, True]
    all_combinations = list(itertools.product(ranges, dyndoms))

    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(10):
        for combination in all_combinations:
            print("Running combination ", combination)
            result = run_planner(combination)
            planning_times[combination].append(result.nanoseconds/1e6)
    print("Execution completed")
    print("Planning times for each combination:")

    combination_means = []
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")
        combination_means.append((combination, np.mean(times)))


    # find the combination with the best mean
    best_combination = sorted(combination_means, key=lambda x: x[1])[0]
    print(best_combination)



    robot_dir = Path(__file__).parents[1] / "resources" / "digit_description"
    server, robot = setup_viser_with_robot(robot_dir, "digit_model_trace_collision_spherized.urdf")
    # robot.update_cfg(a)

    result = run_planner(best_combination[0])
    print(result.nanoseconds/1e6)

    if len(result.path.numpy()):
        add_trajectory(
            server, result.path.numpy(), robot, [], [[]]
        )
        while True:
            pass
