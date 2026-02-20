import numpy as np
import os
from pathlib import Path
import vamp
import meshcat_viz_utils as viz
from fire import Fire


# Starting configuration
a = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# Goal configuration
b = [2.35, 1.0, 0.0, -0.8, 0, 2.5, 0.785]

# Problem specification: a list of sphere centers
problem = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
    [0.35, 0.35, 0.8],
    [0, 0.55, 0.8],
    [-0.35, 0.35, 0.8],
    [-0.55, 0, 0.8],
    [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8],
    [0.35, -0.35, 0.8],
    ]


def get_eef_of_waypoints(waypoints, robot, planner):
    (vamp_module, _, _, _) = vamp.configure_robot_and_planner_with_kwargs(robot, planner)
    eef_poses = []
    for idx, waypoint in enumerate(waypoints):
        eef_pose = vamp_module.eefk(waypoint)
        eef_poses.append(eef_pose.tolist())
    return np.array(eef_poses)


def main(
    obstacle_radius: float = 0.2,
    planner: str = "rrtc",
    **kwargs,
    ):

    (vamp_module, planner_func, plan_settings,
     simp_settings) = (vamp.configure_robot_and_planner_with_kwargs("panda", planner, **kwargs))

    e = vamp.Environment()
    for sphere in problem:
        e.add_sphere(vamp.Sphere(sphere, obstacle_radius))


    # Plan and display
    sampler = vamp_module.halton()
    result = planner_func(a, b, e, plan_settings, sampler)
    simple = vamp_module.simplify(result.path, e, simp_settings, sampler)
    simple.path.interpolate_to_resolution(vamp.panda.resolution())

    print("Planned path with", len(result.path), "waypoints, simplified to", len(simple.path), "waypoints.")

    # display in meshcat
    viz_instance = viz.MeshcatViz()
    viz_instance.init_viz("resources/panda/panda_spherized.urdf", "resources/panda/meshes/")
    viz_instance.clear_all_waypoints()

    # optional: render the waypoints as spheres in meshcat
    eef_poses = get_eef_of_waypoints(simple.path.numpy(), "panda", planner)
    viz_instance.render_eefs(eef_poses)

    viz_instance.animate(simple.path.numpy(), np.arange(0, len(simple.path.numpy()), dtype=np.float64) / 50, loop=True)


if __name__ == "__main__":
    Fire(main)