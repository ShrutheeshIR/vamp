# Third Party
import torch

# cuRobo
from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig, PoseCostMetric
from curobo.types.base import TensorDeviceType

import tqdm
from curobo.geom.types import WorldConfig, Sphere, Cuboid
import numpy as np

import os
import yaml
import json
import time

class MotionPlanningTask:
    def __init__(self):
        self.problem_start = []
        self.problem_end = []
        self.obstacles = []
        self.tsr_lower_bound = []
        self.tsr_upper_bound = []

def load_problems_from_json(json_file_path):
    with open(json_file_path, 'r') as f:
        data = json.load(f)

    problems = []
    for problem_data in data:
        curobo_mask = [1 if abs(float(x)) < 0.1 else 0 for x in problem_data['tsr_lower_bound']]
        curobo_mask = [0, 0, 0] + curobo_mask[:3]

        task = MotionPlanningTask()
        task.problem_start = problem_data['problem_start']
        task.problem_end = problem_data['problem_end']
        task.obstacles = problem_data['cuboid_obstacles']
        task.tsr_lower_bound = curobo_mask
        task.tsr_upper_bound = curobo_mask
        problems.append(task)

    return problems

def plan_task(mp_task: MotionPlanningTask, motion_gen: MotionGen, tensor_args: TensorDeviceType):

    motion_gen.clear_world_cache()

    # Create obstacle
    obstacle_spheres = []
    for i, obs in enumerate(mp_task.obstacles):
        x, y, z, l, w, h = obs
        obstacle_spheres.append(
            Cuboid(
                name=f"obstacle_{i}",
                pose = [x, y, z, 1, 0, 0, 0],
                dims=[l*2, w*2, h*2],
                color=[0, 1.0, 0, 1.0],
            )
        )
    
    world_model = WorldConfig(
        cuboid=obstacle_spheres,
    )

    # convert obstacle to cuboid. This is necessary for collision checking. I think this is stupid.
    cuboid_world = WorldConfig.create_obb_world(world_model)
    motion_gen.update_world(cuboid_world)

    start_state = JointState.from_position(
        tensor_args.to_device([mp_task.problem_start]),
        joint_names=["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"],
    )

    goal_state = JointState.from_position(
        tensor_args.to_device([mp_task.problem_end]),
        joint_names=["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"],
    )
    start_kin_state = motion_gen.rollout_fn.compute_kinematics(start_state)
    ik_start = Pose(
        position=tensor_args.to_device(start_kin_state.ee_pos_seq.clone()),
        quaternion=tensor_args.to_device(start_kin_state.ee_quat_seq.clone()),
    )
    goal_kin_state = motion_gen.rollout_fn.compute_kinematics(goal_state)
    ik_goal = Pose(
        position=tensor_args.to_device(goal_kin_state.ee_pos_seq.clone()),
        quaternion=tensor_args.to_device(goal_kin_state.ee_quat_seq.clone()),
    )

    valid_query, status = motion_gen.check_start_state(start_state)
    if not valid_query:
        # print("\033[91m" + "Invalid start state" + "\033[0m")
        # print(status)
        return -1

    valid_query, status = motion_gen.check_start_state(goal_state)
    if not valid_query:
        # print("\033[91m" + "Invalid goal state" + "\033[0m")
        # print(status)
        return -1

    pose_cost_metric = PoseCostMetric(
        hold_partial_pose=True,
        hold_vec_weight=motion_gen.tensor_args.to_device([int(lb) for lb in mp_task.tsr_lower_bound]),
    )
    # print(pose_cost_metric)
    # pose_cost_metric = PoseCostMetric(
    #     hold_partial_pose=True,
    #     hold_vec_weight=motion_gen.tensor_args.to_device([1, 1, 1, 0, 0, 1]),

    # )

    motion_gen_config = MotionGenPlanConfig(
            max_attempts=20,  # 20,
            enable_graph_attempt=1,
            disable_graph_attempt=10,
            enable_finetune_trajopt=True,
            enable_graph=False,
            timeout=60,
            enable_opt=True,
            need_graph_success=False,
            parallel_finetune=True,
            finetune_dt_scale=0.8,
            check_start_validity=True,
        )
    # motion_gen_config = MotionGenPlanConfig(
    #     enable_graph=True, 
    #     enable_opt=False,
    #     use_nn_ik_seed=False,
    #     need_graph_success=False,
    #     max_attempts=20,
    #     timeout=10.0,
    #     enable_finetune_trajopt=False,
    #     parallel_finetune=False,
    #     finetune_attempts=0,
    #     check_start_validity=True,
    # )
    motion_gen_config.pose_cost_metric = pose_cost_metric
    # result = motion_gen.plan_single_js(start_state, goal_state, motion_gen_config)



    # print(ik_start, ik_goal)
    projected_pose = ik_goal.compute_local_pose(ik_start)
    # print("Projected pose: ", projected_pose)


    # ik_goal = Pose(
    #     position=tensor_args.to_device(torch.tensor([0.55, 0.4, 0.5])),
    #     quaternion=tensor_args.to_device(torch.tensor([0.5, -0.5, 0.5, 0.5])),
    # )

    # ik_goal = Pose(
    #     position=tensor_args.to_device(
    #         torch.tensor([0.3493, -0.6499,  0.3449])
    #     ),
    #     quaternion=tensor_args.to_device(
    #         torch.tensor([4.8889e-05,  1.0000e+00,  2.6360e-04, -2.9853e-04])
    #     ),
    # )


    # print(ik_goal)
    result = motion_gen.plan_single(start_state, ik_goal, motion_gen_config)
    # cmd_plan = result.get_interpolated_plan()
    # cmd_plan = motion_gen.get_full_js(cmd_plan)
    # print(cmd_plan.position[-1, :7].cpu().numpy())
    # stop
    return result

def load_large_json_with_extra(filepath):
    with open(filepath, 'r') as f:
        data = f.read()
    
    decoder = json.JSONDecoder()
    pos = 0
    results = []
    problems = []
    
    while pos < len(data):
        # Skip whitespace
        if data[pos].isspace():
            pos += 1
            continue
        try:
            obj, pos = decoder.raw_decode(data, pos)
            results.append(obj)
        except json.JSONDecodeError:
            break # Or handle specific corruption


    for result in results:
        for obj in result:
            # task = MotionPlanningTask()
            # task.problem_start = obj['problem_start']
            # task.problem_end = obj['problem_end']
            # task.obstacles = obj['cuboid_obstacles']
            # task.tsr_lower_bound = [1 if abs(float(x)) > 0.1 else 0 for x in obj['tsr_lower_bound']]
            # task.tsr_upper_bound = [1 if abs(float(x)) > 0.1 else 0 for x in obj['tsr_upper_bound']]
            problems.append(obj)
    
    # save this fixed json to the file
    # with open(filepath, 'w') as f:
    #     json.dump(problems, f, indent=4)



    problems = []
    for result in results:
        for obj in result:
            curobo_mask = [1 if abs(float(x)) < 0.1 else 0 for x in obj['tsr_lower_bound']]
            curobo_mask = curobo_mask[3:] + curobo_mask[:3]
            task = MotionPlanningTask()
            task.problem_start = obj['problem_start']
            task.problem_end = obj['problem_end']
            task.obstacles = obj['cuboid_obstacles']
            task.tsr_lower_bound = curobo_mask
            task.tsr_upper_bound = curobo_mask
            problems.append(task)


    return problems


class SolvedResult:
    def __init__(self, success, plan=None, graph_time=0, solve_time=0, trajopt_time=0, finetune_time=0, num_cuboid_obstacles=0, path_cost=0.0):
        self.success = success
        self.plan = plan
        self.total_solve_time = graph_time + solve_time + trajopt_time + finetune_time
        self.num_cuboid_obstacles = num_cuboid_obstacles
        self.path_cost = path_cost


def save_solved_results_to_plot_data(solved_results, save_path):
    plot_data = []
    for result in solved_results:
        plot_data.append({
            "success": result.success,
            "total_solve_time": result.total_solve_time * 1000.0,  # convert to milliseconds
            "num_cuboid_obstacles": result.num_cuboid_obstacles,
            "plan": result.plan.tolist() if result.plan is not None else None,
            "path_cost": result.path_cost,
        })
    with open(save_path, 'w') as f:
        json.dump(plot_data, f, indent=4)


def main():
    tensor_args = TensorDeviceType(device=torch.device("cuda:0"))

    # We need to create dummpy obstacle to initialize the motion generator, so it can first create the collision cache.
    # Create obstacle
    obstacle_sphere = Sphere(
        name="dummy_obstacle",
        radius=0.2,
        pose=[0.506, 0.0, 10, 1, 0, 0, 0],
        color=[0, 1.0, 0, 1.0],
    )
    world_model = WorldConfig(
    sphere=[obstacle_sphere],
    )

    # convert obstacle to cuboid. This is necessary for collision checking. I think this is stupid.
    cuboid_world = WorldConfig.create_obb_world(world_model)

    motion_gen_config = MotionGenConfig.load_from_robot_config(
        "/home/ros/curobo/src/curobo/content/configs/robot/franka.yml",
        cuboid_world,
        interpolation_dt=0.02,
        project_pose_to_goal_frame=True,
        collision_cache={"obb": 100},
        ee_link_name="panda_hand",
        position_threshold=0.01,  # 1 cm
        rotation_threshold=0.01,
    )

    motion_gen = MotionGen(motion_gen_config)
    motion_gen.warmup()

    num_of_success = 0
    num_of_tasks = 0
    total_time_of_success_case = 0

    #  do a dummy task first
    dummy_task = MotionPlanningTask()
    dummy_task.problem_start = [1.01600, 0.68800, 0.08700, -1.28100, -0.06000, 1.95500, 1.89100]
    dummy_task.problem_end = [-0.91508937, 0.6985612, -0.23231459, -1.2822142, 0.16026999, 1.962646, -0.37113136]
    dummy_task.obstacles = [
        [0.56, 0, 0.450, 0.1],
        [0.1, 0, 0.7, 0.1],
        [0, 0.55, 0.25, 0.1],
        [-0.55, 0, 0.25, 0.1],
        [-0.35, -0.35, 0.25, 0.1],
        [0, -0.55, 0.25, 0.1],
        [0.35, 0.35, 0.8, 0.1],
        [0, 0.55, 0.8, 0.1],
        [-0.35, 0.35, 0.8, 0.1],
        [-0.55, 0, 0.8, 0.1],
        [-0.35, -0.35, 0.8, 0.1],
        [0, -0.55, 0.8, 0.1],
        [0.35, -0.35, 0.8, 0.1]
    ]
    # np.savetxt("/src/spheres.txt", dummy_task.obstacles, fmt="%.5f", delimiter=",")
    dummy_task.tsr_lower_bound = [0, 0, 0, 0, 1, 0]
    dummy_task.tsr_upper_bound = [0, 0, 0, 0, 1, 0]

    # for _ in range(2):
    #     result = plan_task(dummy_task, motion_gen, tensor_args)
    #     print(result.success, result.graph_time, result.solve_time, result.trajopt_time, result.finetune_time)
    #     if result.success:
    #         cmd_plan = result.get_interpolated_plan()
    #         cmd_plan = motion_gen.get_full_js(cmd_plan)
    #         waypoints = cmd_plan.position[:, :7].cpu().numpy()
    #         print(waypoints[-1])
    #         np.savetxt("/src/dummy_plan.txt", waypoints, fmt="%.5f", delimiter=",")

    tasks = load_problems_from_json("scripts/cpp/benchmarks/line_plane_benchmark_problems/tsr_panda_problems_curobo_cuboid_prespecified_line_curobo_likes.json")
    # print(tasks[0])
    print(f"Loaded {len(tasks)} tasks from json file.")

    solved_plans: list[SolvedResult] = []

    for i, task in tqdm.tqdm(enumerate(tasks), total=len(tasks)):
        # print(f"Planning task {i+1}/{len(tasks)}")
        result = plan_task(task, motion_gen, tensor_args)
        np.savetxt("/src/cuboids.txt", task.obstacles, fmt="%.5f", delimiter=",")
        if result == -1:
            # just interpolate between start and goal configs and write
            start_config = task.problem_start
            goal_config = task.problem_end
            waypoints = np.linspace(start_config, goal_config, num=50)
            # np.savetxt("/src/dummy_plan.txt", waypoints, fmt="%.5f", delimiter=",")
            # time.sleep(5.0)
            continue
        if result.success:
            # print(f"Task {i+1} success: {result.success}")
            num_of_success += 1
            cmd_plan = result.get_interpolated_plan()
            cmd_plan = motion_gen.get_full_js(cmd_plan)
            waypoints = cmd_plan.position[:, :7].cpu().numpy()
            # np.savetxt("/src/dummy_plan.txt", waypoints, fmt="%.5f", delimiter=",")

            # time.sleep(5.0)
        else:
            # just interpolate between start and goal configs and write
            start_config = task.problem_start
            goal_config = task.problem_end
            waypoints = np.linspace(start_config, goal_config, num=50)
            # np.savetxt("/src/dummy_plan.txt", waypoints, fmt="%.5f", delimiter=",")
            # time.sleep(5.0)
            # continue

        path_length = torch.sum(
            torch.linalg.norm(
                (
                    torch.roll(result.optimized_plan.position, -1, dims=-2)
                    - result.optimized_plan.position
                )[..., :-1, :],
                dim=-1,
            )
        ).item()

        solved_plans.append(SolvedResult(
            success=result.success.squeeze().cpu().numpy().item(),
            plan=cmd_plan.position[:, :7].cpu().numpy() if result.success else None,
            graph_time=result.graph_time,
            solve_time=result.solve_time,
            trajopt_time=result.trajopt_time,
            finetune_time=result.finetune_time,
            num_cuboid_obstacles=len(task.obstacles),
            path_cost=path_length
        ))

        num_of_tasks += 1
    print(f"Success rate: {num_of_success}/{num_of_tasks}, Average time of success case: {total_time_of_success_case/num_of_success if num_of_success > 0 else 0:.2f} seconds")
    save_solved_results_to_plot_data(solved_plans, "scripts/cpp/benchmarks/line_plane_benchmark_problems/tsr_panda_problems_curobo_cuboid_plane_curobo_no_ori_results.json")

if __name__ == "__main__":
    main()