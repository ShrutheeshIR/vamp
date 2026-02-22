import numpy as np
import os
from pathlib import Path
import json
import vamp
from fire import Fire
from scipy.spatial.transform import Rotation as R
import itertools

np.set_printoptions(suppress = True)
# Starting configuration
a = [1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100]

# Goal configuration
b = [-1.18400, 0.68900, 0.15400, -1.27400, -0.10600, 1.95500, -0.24000]

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

    fka = vamp_module.eefk(a)
    fkb = vamp_module.eefk(b)


    print(R.from_matrix(fka[0][:3, :3]).as_quat(scalar_first = True), fka[0][:3, 3])
    print(R.from_matrix(fkb[0][:3, :3]).as_quat(scalar_first = True), fkb[0][:3, 3])

    tsr = vamp_module.TaskSpaceConstraint(
        [[1, 0, 0, 0, 0, 0, 0]],
        [[0, 1,0,0,   0.3486, 0.647752, 0.2399]],
        [-10.01, -10.01, -0.01, -10.01, -10.01, -10.01],
        [10.01, 10.01, 0.01, 0.01, 10.01, 10.01]
    )
    constraints = vamp_module.Composable_T(tsr)

    e = vamp.Environment()

    sampler = vamp_module.halton()

    print("Running planner")
    result = planner_func(a, b, e, plan_settings, constraints, sampler)

    simple = vamp_module.simplify_with_constraints(result.path, e, constraints, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())
    ranges = [0.1, 0.2, 0.5, 0.75]
    dyndoms = [False, True]
    all_combinations = list(itertools.product(ranges, dyndoms))
    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(1):
        for combination in all_combinations:
            plan_settings.range = combination[0]
            plan_settings.dynamic_domain = combination[1]
            print("Running planner")
            result = planner_func(a, b, e, plan_settings, constraints, sampler)
            print(result.nanoseconds / 1e3)
            planning_times[combination].append(result.nanoseconds/1e6)

    print("Execution completed")
    print("Planning times for each combination:")
    for combination, times in planning_times.items():
        print(f"Combination {combination}: {np.mean(times)} ms {np.std(times)} ms {np.min(times)} ms {np.max(times)} ms {np.median(times)} ms")


    # display
    while True:
        continue


if __name__ == "__main__":
    Fire(main)
