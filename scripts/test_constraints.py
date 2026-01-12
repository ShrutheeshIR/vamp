import numpy as np
import os
from pathlib import Path
import time
import vamp
from fire import Fire

def main(
    **kwargs,
):
    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("panda", "crrtc", **kwargs)
    )
    tsr = vamp_module.TaskSpaceConstraint([[0, 1,0,0,   0.3486, 0.647752, 0.2399]],[[1, 0, 0, 0, 0, 0, 0]], [-0.01, -10.01, -0.01, -0.01, -0.01, -0.01], [0.01, 10.01, 0.01, 0.01, 0.01, 0.01])

    constraints = vamp_module.Composable_TaskSpaceConstraint(tsr)
    e = vamp.Environment()
    sampler = vamp_module.halton()

    start = [1.016, 0.688, 0.087, -1.281, -0.06, 1.955, 1.891]
    goal = [-1.184, 0.689, 0.154, -1.274, -0.106, 1.955, -0.24]

    t1 = time.perf_counter_ns()
    result = planner_func(start, goal, e, plan_settings, constraints, sampler)
    print((time.perf_counter_ns() - t1) / 1e6)


if __name__ == '__main__':
    Fire(main)
