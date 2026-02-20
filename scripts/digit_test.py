import numpy as np
import os
from pathlib import Path
import json
import vamp
from fire import Fire
from scipy.spatial.transform import Rotation as R
import itertools
import meshcat_viz as viz

import mjcf_parser


def get_eef_of_waypoints(waypoints):

    (vamp_module, _, _, _) = vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc")
    eef_poses = []
    for waypoint in waypoints:
        eef_pose = [pose.tolist() for pose in vamp_module.eefk(waypoint)]
        eef_poses.extend(eef_pose)
    return eef_poses

def run_planner(
    combination,
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,
    **kwargs,
):

    start = [
        0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000, 
        0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
        -0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
        -0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
        0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000
    ]

    box_top_shelf_pickup = [
        0.00799012, -0.02020776,  0.00473523, -0.00283502, -0.01516565, -0.00604922,                                                                                                                                                                                                             
        0.38425133,  0.0008674,   0.29416478,  0.32168943, -0.00645173, -0.296634,                                                                                                                                                                                                               
        0.02625096,  0.01125011,  0.0887334,  -0.26175958, -0.00802566,  0.11402924,                                                                                                                                                                                                             
        -0.37266037, -0.00365927, -0.2966428,  -0.3174328,   0.01091318,  0.27964896,                                                                                                                                                                                                             
        -0.13281101,  0.00645007,  0.16188946,  0.27721465,  0.06585242, -0.09529761,
    ]

    rack_2 = [
        0.00927,-0.01219,-0.47283,-0.01411,-0.02955,-0.00937,0.40005,0.01408,-0.23717,-0.84704,-0.02429,0.90190,-0.48186,0.04543,-0.17059,-0.36363,0.06204,1.09005,-0.36164,-0.00267,0.22962,0.84538,0.02448,-0.90900,0.40052,0.03680,-0.29646,0.37895,-0.15478,-1.16821
    ]

    rack_3 = [
        0.00927,-0.01219,-0.47283,-0.01411,-0.02955,-0.00937,0.40005,0.01408,-0.23717,-0.84704,-0.02429,0.90190,-0.48186,0.04543,-0.17059,-0.36363,0.06204,1.09005,-0.36164,-0.00267,0.22962,0.84538,0.02448,-0.90900,0.40052,0.03680,-0.29646,0.37895,-0.15478,-1.16821
    ]


    polygon_points = [
        0.01, -0.01, 0.01, 0.03, -0.03, 0.03, -0.03, -0.01
    ]

    bimanual_limit_lower_bound = [
        -0.001, -0.001, -0.001, -0.1, -10.1, -10.1
    ]
    bimanual_limit_upper_bound = [
        0.001, 0.001, 0.001, 0.1, 10.1, 10.1
    ]

    tsr_lower_bound = [
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -0.001, -0.001, -0.001, -0.1, -0.1, -0.1, 
        -0.001, -0.001, -0.001, -0.1, -0.1, -0.1
    ]

    tsr_upper_bound = [
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 
        0.001, 0.001, 0.001, 0.1, 0.1, 0.1
    ]


    eef_transforms = [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [0.59, 0.38, 0.4, 0.59 , -0.02070, 0.06015, -0.95335], [0.61, -0.36, 0.35, -0.61 , -0.02228, -0.11609, -0.94832]];
    eef_transforms_ref_frame_w_world = [[1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0]];

    bimanual_transform = [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356];




    (vamp_module, planner_func, plan_settings, simp_settings) = (
        vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc", **kwargs)
    )

    print(vamp_module.joint_names())



    bimanual_constraint = vamp_module.BimanualTaskSpaceConstraint(
        bimanual_transform,
        bimanual_limit_lower_bound,
        bimanual_limit_upper_bound
    )
    com_constraint = vamp_module.CoMTaskSpaceConstraint(
        polygon_points,
    )


    feet_tsr_constraint = vamp_module.FeetTaskSpaceConstraint(
        eef_transforms_ref_frame_w_world,
        eef_transforms,
        tsr_lower_bound,
        tsr_upper_bound
    )

    closed_link_constraint = vamp_module.ClosedLinkConstraint()


    transport_constraints = vamp_module.Composable_F_C_CL_B(feet_tsr_constraint, com_constraint, closed_link_constraint, bimanual_constraint)
    non_transport_constraints = vamp_module.Composable_F_C_CL(feet_tsr_constraint, com_constraint, closed_link_constraint)



    e = vamp.Environment()
    env_geoms = mjcf_parser.parse_mjcf('resources/environments/cuboids/wooden_shelf.xml')
    for geom in env_geoms:
        if geom.type == mjcf_parser.GeomType.BOX:
            e.add_cuboid(vamp.Cuboid([geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z], [0, 0, 0], [geom.size.x, geom.size.y, geom.size.z]))

    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, 0])
    attachment = vamp.Attachment(tf)


    attach_spheres = [
        [-0.02, 0.0, -0.09],
        [-0.02, -0.02, -0.18],
        [-0.02, -0.04, -0.26],
        [-0.02, 0.05, -0.09],
        [-0.02, 0.03, -0.18],
        [-0.02, 0.01, -0.26],
        [0.04, 0.0, -0.09],
        [0.04, -0.02, -0.18],
        [0.04, -0.04, -0.26],
        [0.04, 0.05, -0.09],
        [0.04, 0.03, -0.18],
        [0.04, 0.01, -0.26],
    ]

    attachment.add_spheres(
        [
            vamp.Sphere(sphere, 0.05) for sphere in attach_spheres
        ]
    )
    e.attach(attachment, 0);

    sampler = vamp_module.halton()

    plan_settings.range = combination[0]
    plan_settings.dynamic_domain = combination[1]

    plan_settings.max_iterations = 100000;
    plan_settings.descend_rate = 1.0;

    plan_settings.radius = 10.0;
    plan_settings.num_projection_iterations = 10;
    plan_settings.insert_all_to_tree = True;

    # result = planner_func(start, box_top_shelf_pickup, e, plan_settings, non_transport_constraints, sampler)
    result = planner_func(box_top_shelf_pickup, rack_2, e, plan_settings, transport_constraints, sampler)

    simple = vamp_module.simplify_with_constraints(result.path, e, transport_constraints, simp_settings, sampler)
    # simple = vamp_module.simplify_with_constraints(result.path, e, non_transport_constraints, simp_settings, sampler)

    return result, simple


if __name__ == '__main__':
    # ranges = [0.5, 0.75, 1.0, 1.5]
    # dyndoms = [False, True]
    ranges = [0.75, 0.5, 1.0]
    dyndoms = [True, False]

    all_combinations = list(itertools.product(ranges, dyndoms))

    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(1):
        for combination in all_combinations:
            print("Running combination ", combination)
            result, simple = run_planner(combination)
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



    viz.init_viz()
    viz.clear_all_waypoints()


    result, simple = run_planner(best_combination[0])
    print(result.nanoseconds/1e6)

    if len(simple.path.numpy()):
        simple.path.interpolate_to_resolution(vamp.digit.resolution())

        traj = simple.path.numpy()
        eef_poses = get_eef_of_waypoints(traj)
        
        viz.render_eefs(np.array(eef_poses))
        viz.animate(traj, np.arange(0, len(traj), dtype=np.float64) / 10, loop=True)


        while True:
            pass
