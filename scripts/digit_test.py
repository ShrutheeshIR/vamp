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

    start = [0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000,
    0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
        -0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
    -0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
        0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000]
    
    goal = [-0.00274,0.00213,-0.00372,-0.00098,-0.00245,-0.00053,0.36473,-0.00404,0.29244,0.32056,-0.00824,-0.29309,0.09298,-0.01021,0.08865,-0.26330,-0.00645,0.11542,-0.36414,0.00349,-0.29690,-0.32320,0.00488,0.29348,-0.09189,0.01269,0.16190,0.27658,0.06754,-0.09473]



    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc", **kwargs)
    )
    print("Created vamp module")
    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356],
        [-0.001, -0.001, -0.001, -0.1, -10.1, -10.1],
        [0.001, 0.001, 0.001, 0.1, 10.1, 10.1]
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        [0.03, -0.075, 0.03, 0.075, -0.04, 0.075, -0.04, -0.075],
    )


    feet_tsr_constraint = vamp_module.TaskSpaceConstraint(
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [0.603, 0.36, 0.36, 0.603, -0.04302,  0.10080, -0.96013], [0.603, -0.36, 0.36, -0.603 , -0.04288, -0.09895, -0.96033]],
        [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]],
        [-10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, -0.001, -0.001, -0.001, -0.05, -0.05, -0.05, -0.001, -0.001, -0.001, -0.05, -0.05, -0.05],
        [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 0.001, 0.001, 0.001, 0.05, 0.05, 0.05, 0.001, 0.001, 0.001, 0.05, 0.05, 0.05]
    )


    constraints = vamp_module.Composable_T_C_B(feet_tsr_constraint, com_constraint, bimanual_constraint)


    problem_cuboids = np.array([[0.65,0.03,-0.925,   0.15,0.35,0.025],
            [0.65,0.03,-0.525 ,  0.15,0.35,0.025],
            [0.65,0.03,-0.125 ,  0.15,0.35,0.025],
            [0.65,0.03,0.25  , 0.15,0.35,0.025],
            [0.815,0.405,-0.35 ,  0.025,0.025,0.6],
            [0.815,-0.345,-0.35 ,  0.025,0.025,0.6],
            [0.485,0.405,-0.35  , 0.025,0.025,0.6],
            [0.485,-0.345,-0.35  , 0.025,0.025,0.6]])


    e = vamp.Environment()
    # problem_cuboids = np.loadtxt('resources/environments/cuboids/humanoid_digit_shelf.txt', delimiter = ",")
    for cuboid in problem_cuboids:
        e.add_cuboid(vamp.Cuboid(cuboid[:3], [0, 0, 0], cuboid[3:6]))

    # e.attach(attachment, 0)
    sampler = vamp_module.halton()

    # result = planner_func(start, goal, e, plan_settings, constraints, sampler)

    # np.set_printoptions(precision = 4, suppress = True)
    # print(np.array(vamp_module.eefk(start)))
    # print(np.array(vamp_module.eefk(goal)))

    plan_settings.range = combination[0]
    plan_settings.dynamic_domain = combination[1]
    plan_settings.max_iterations = 100000
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
