import numpy as np
import os
from pathlib import Path
import json
import vamp
from fire import Fire
import itertools
import meshcat_viz as viz
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import mjcf_parser


start = [
    0.0287480000000000, -0.0189052000000000, -0.0145299000000000, 0.00967345000000000, 0.00825856000000000, -0.00215209000000000, 
    0.397209000000000, -0.00820680000000000, 0.284002000000000, 0.290042000000000, -0.0213535000000000, -0.235481000000000, -0.0319210000000000, 0.00127043000000000,
    -0.0782953000000000, 1.04159000000000, 0.0284093000000000, -0.0123198000000000,
    -0.358284000000000, -0.0133610000000000, -0.278441000000000, -0.268447000000000, 0.0177607000000000, 0.212721000000000, -0.0692397000000000, -0.000343739000000000,
    0.0689620000000000, -1.23829000000000, 0.0369306000000000, -0.00862385000000000
]

box_top_shelf_pregrasp = [
    1.11889839e-02, -1.99530125e-02,  5.81884384e-03, -3.29416106e-03, -1.42211439e-02, -5.99248195e-03,  
    3.84082675e-01,  5.58406231e-04, 2.98647016e-01,  3.22717577e-01, -3.98048153e-03, -3.00990015e-01, 1.07899308e-02,  1.02880923e-02,  
    3.03654131e-02,  1.43816188e-01, -0.139, -5.22131145e-01,
     -3.72209966e-01, -3.30008916e-03, -2.98724055e-01, -3.18776041e-01,  8.88650957e-03,  2.83709198e-01, -1.18316561e-01,  6.75600534e-03,  
    4.73398268e-02, -1.23632416e-01,  0.139,  4.99689251e-01
]


box_top_shelf_pickup = [
    0.0111889839,-0.0199530125,0.00581884384,-0.00329416106,-0.0142211439,-0.00599248195,0.384082675,0.000558406231,0.298647016,0.322717577,-0.00398048153,-0.300990015,0.0107899308,0.0102880923,0.0303654131,0.143816188,0.246,-0.522131145,-0.372209966,-0.00330008916,-0.298724055,-0.318776041,0.00888650957,0.283709198,-0.118316561,0.00675600534,0.0473398268,-0.123632416,-0.217,0.499689251
]

rack_2 = [
0.0160, -0.0108, -0.4697, -0.0161, -0.0261, -0.0077, 0.4025, 0.0070, -0.2220, -0.8470, -0.0190, 0.8941, -0.5331, 0.0436, 0.1756, 0.3280, 0.1925, -0.1410, -0.3597, -0.0049, 0.2170, 0.8447, 0.0197, -0.9012, 0.4206, 0.0378, 0.0701, -0.2473, -0.3055, 0.1341
]

rack_2_release = [
0.0160, -0.0108, -0.4697, -0.0161, -0.0261, -0.0077, 
0.4025, 0.0070, -0.2220, -0.8470, -0.0190, 0.8941, -0.5331, 0.0436, 
0.1756, 0.3280, 0.01925, -0.1410, 
-0.3597, -0.0049, 0.2170, 0.8447, 0.0197, -0.9012, 0.4206, 0.0378, 
0.0701, -0.2473, -0.0055, 0.1341
]


rack_3 = [
    0.0207, -0.0116, -0.4709, -0.0138, -0.0174, -0.0089, 0.4015, 0.0084, -0.2226, -0.8477, -0.0198, 0.8960, -0.5230, 0.0441, 0.0881, 0.7864, -0.3927, -0.8881, -0.3608, -0.0046, 0.2179, 0.8458, 0.0208, -0.9040, 0.4115, 0.0377, -0.3408, -0.0583, -0.9770, 0.2251
]

attach_spheres = [
    [-0.05, 0.0, -0.04],
    [-0.05, -0.02, -0.07],
    [-0.05, -0.04, -0.11],
    [-0.05, -0.06, -0.13],
    [-0.05, 0.05, -0.04],
    [-0.05, 0.03, -0.07],
    [-0.05, 0.01, -0.11],
    [-0.05, -0.01, -0.13],
    [-0.02, 0.0, -0.04],
    [-0.02, -0.02, -0.07],
    [-0.02, -0.04, -0.11],
    [-0.02, -0.06, -0.13],
    [-0.02, 0.05, -0.04],
    [-0.02, 0.03, -0.07],
    [-0.02, 0.01, -0.11],
    [-0.02, -0.01, -0.13],
    [0.01, 0.0, -0.04],
    [0.01, -0.02, -0.07],
    [0.01, -0.04, -0.11],
    [0.01, -0.06, -0.13],
    [0.01, 0.05, -0.04],
    [0.01, 0.03, -0.07],
    [0.01, 0.01, -0.11],
    [0.01, -0.01, -0.13],
    [0.04, 0.0, -0.04],
    [0.04, -0.02, -0.07],
    [0.04, -0.04, -0.11],
    [0.04, -0.06, -0.13],
    [0.04, 0.05, -0.04],
    [0.04, 0.03, -0.07],
    [0.04, 0.01, -0.11],
    [0.04, -0.01, -0.13],
    [0.07, 0.0, -0.04],
    [0.07, -0.02, -0.07],
    [0.07, -0.04, -0.11],
    [0.07, -0.06, -0.13],
    [0.07, 0.05, -0.04],
    [0.07, 0.03, -0.07],
    [0.07, 0.01, -0.11],
    [0.07, -0.01, -0.13],
]

def convert_trajectory_eef_to_controller_format(trajectory, eef_waypoints):
    # create a 8 + 7 + 7 + 7 = 29 dim waypoint
    # the first 8 values are left and right arms joint positions
    # next 7 is the floating base pose (x, y, z, qw, qx, qy, qz)
    # next 7 is left foot pose (x, y, z, qw, qx, qy, qz)
    # next 7 is right foot pose (x, y, z, qw, qx, qy, qz)
    # eef_waypoints is a n x 4 x 4 x 4 array where the first 4 is number of eefs, n is number of waypoints, and the last 4x4 is the transform of the eef in world frame

    print(eef_waypoints.shape)

    output = np.zeros((len(trajectory), 29))
    # arm joints are 14:18 and, 26:29
    output[:, :8] = trajectory[:, [14, 15, 16, 17, 26, 27, 28, 29]]
    # floating base is 0:6. Need to convert euler to quat for orientation

    def to_quat(euler):
        r = R.from_euler('xyz', euler, degrees=False)
        return r.as_quat() # returns in (x, y, z, w) format

    def to_quat_from_rotmatrix(rotmat):
        r = R.from_matrix(rotmat)
        return r.as_quat() # returns in (x, y, z, w) format

    output[:, 8:11] = trajectory[:, 0:3]
    output[:, 11:15] = np.apply_along_axis(to_quat, 1, trajectory[:, 3:6])
    # left foot is eef_waypoints[:, 2] as a 4x4 matrix
    # need to convert it to (x, y, z, qw, qx, qy, qz) format
    output[:, 15:18] = eef_waypoints[:, 2, :3, 3] # position
    output[:, 18:22] = np.array([to_quat_from_rotmatrix(eef_waypoints[i, 2, :3, :3]) for i in range(len(eef_waypoints))]) # orientation
    # right foot is eef_waypoints[:, 3] as a 4x4 matrix
    output[:, 22:25] = eef_waypoints[:, 3, :3, 3] # position
    output[:, 25:29] = np.array([to_quat_from_rotmatrix(eef_waypoints[i, 3, :3, :3]) for i in range(len(eef_waypoints))]) # orientation

    return output




def get_eef_of_waypoints(waypoints):

    (vamp_module, _, _, _) = vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc")
    eef_poses = []
    for waypoint in waypoints:
        eef_pose = [pose.tolist() for pose in vamp_module.eefk(waypoint)]
        eef_poses.append(eef_pose)
    return np.array(eef_poses)

def run_planner(
    start,
    goal,
    combination,
    constraint = 0,
    obstacle_radius: float = 0.15,
    attachment_radius: float = 0.05,
    attachment_offset: float = 0.02,
    **kwargs,
):



    polygon_points = [
        0.005, -0.01, 0.005, 0.03, -0.05, 0.03, -0.05, -0.01
    ]

    bimanual_limit_lower_bound = [
        -0.005, -0.005, -0.005, -0.1, -0.1, -10.1
    ]
    bimanual_limit_upper_bound = [
        0.005, 0.005, 0.005, 0.1, 0.1, 10.1
    ]

    tsr_lower_bound = [
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -10.0, -10.0, -10.0, -10.0, -10.0, -10.0, 
        -0.001, -0.001, -0.001, -0.03, -0.03, -0.03, 
        -0.001, -0.001, -0.001, -0.03, -0.03, -0.03
    ]

    tsr_upper_bound = [
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 
        0.001, 0.001, 0.001, 0.03, 0.03, 0.03, 
        0.001, 0.001, 0.001, 0.03, 0.03, 0.03
    ]


    eef_transforms = [[1, 0,0,0,   0, 0, 0], [1, 0,0,0,   0.0, 0.0, 0.0], [0.59, 0.38, 0.4, 0.59 , -0.02070, 0.06015, -0.95335], [0.61, -0.36, 0.35, -0.61 , -0.02228, -0.11609, -0.94832]];
    eef_transforms_ref_frame_w_world = [[1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0, 0]];

    bimanual_transform = [0.05, 0.95, 0.00000, 0.3, 0.06308, -0.00, -0.17395];




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
            if 'transport' in geom.geom_name and constraint!= 0:
                continue
            e.add_cuboid(vamp.Cuboid([geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z], [0, 0, 0], [geom.size.x, geom.size.y, geom.size.z]))

    tf = np.identity(4)
    tf[:3, 3] = np.array([0, 0, 0])
    attachment = vamp.Attachment(tf)

    attachment.add_spheres(
        [
            vamp.Sphere(sphere, 0.02) for sphere in attach_spheres
        ]
    )
    if constraint != 0:
        e.attach(attachment, 0);

    sampler = vamp_module.halton()

    plan_settings.range = combination[0]
    plan_settings.dynamic_domain = combination[1]

    plan_settings.max_iterations = 100000;
    plan_settings.descend_rate = 1.0;

    plan_settings.radius = 10.0;
    plan_settings.num_projection_iterations = 10;
    plan_settings.insert_all_to_tree = True;

    if constraint == 0:
        task_constraint = non_transport_constraints
    else:
        task_constraint = transport_constraints

    print(task_constraint.distanceToConstraint(np.array(start)))
    print(task_constraint.distanceToConstraint(np.array(goal)))

    c1 = task_constraint.projectConfiguration(np.array(start), 0, 10.0, 0.75, 50, True)
    c2 = task_constraint.projectConfiguration(np.array(goal), 0, 10.0, 0.75, 50, True)

    # print c1 with commas
    print("c1: ", ", ".join([f"{x:.4f}" for x in c1]))
    print("c2: ", ", ".join([f"{x:.4f}" for x in c2]))


    result = planner_func(start, goal, e, plan_settings, task_constraint, sampler)
    simple = vamp_module.simplify_with_constraints(result.path, e, task_constraint, simp_settings, sampler)

    return result, simple


if __name__ == '__main__':
    ranges = [0.5, 0.75, 1.0, 1.5]
    dyndoms = [False, True]
    # ranges = [0.75]
    # dyndoms = [False]

    all_combinations = list(itertools.product(ranges, dyndoms))

    planning_times = {
        combination : [] for combination in all_combinations
    }
    for _ in range(1):
        for combination in all_combinations:
            print("Running combination ", combination)
            # result, simple = run_planner(start, box_top_shelf_pickup, combination, constraint = (start == box_top_shelf_pickup))
            result, simple = run_planner(rack_3, rack_3, combination, constraint = 1)
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
    # best_combination = [(0.75, False)]


    viz.init_viz()
    viz.clear_all_waypoints()


    env_geoms = mjcf_parser.parse_mjcf('resources/environments/cuboids/wooden_shelf.xml')
    env_cuboids = [[geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z, 0, 0, 0, geom.size.x * 2.0, geom.size.y * 2.0, geom.size.z*2.0] for geom in env_geoms if geom.type == mjcf_parser.GeomType.BOX]
    viz.add_cuboids(env_cuboids, colors=(90, 60, 0))

    attach_sphere_w_r = [[x, y, z, 0.02] for x, y, z in attach_spheres] 
    viz.set_attach_object_to_robot(attach_sphere_w_r)
    # for geom in env_geoms:
    #     if geom.type == mjcf_parser.GeomType.BOX:
    #         viz.add_cuboids([
    #             [geom.world_pose.pos.x, geom.world_pose.pos.y, geom.world_pose.pos.z, 0, 0, 0, geom.size.x * 2.0, geom.size.y * 2.0, geom.size.z*2.0]
    #         ], colors=(150, 75, 0))



    result1, simple1 = run_planner(start, box_top_shelf_pregrasp, best_combination[0], constraint = 0)
    simple1.path.interpolate_to_resolution(vamp.digit.resolution())
    traj1 = simple1.path.numpy()

    result2, simple2 = run_planner(box_top_shelf_pregrasp, box_top_shelf_pickup, best_combination[0], constraint = 0)
    simple2.path.interpolate_to_resolution(vamp.digit.resolution())
    traj2 = simple2.path.numpy()


    result3, simple3 = run_planner(box_top_shelf_pickup, rack_2, best_combination[0], constraint = 2)
    simple3.path.interpolate_to_resolution(vamp.digit.resolution())
    traj3 = simple3.path.numpy()

    result4, simple4 = run_planner(rack_2, rack_2_release, best_combination[0], constraint = 0)
    simple4.path.interpolate_to_resolution(vamp.digit.resolution())
    traj4 = simple4.path.numpy()

    # result5, simple5 = run_planner(rack_2_release, rack_3, best_combination[0], constraint = 0)
    # simple4.path.interpolate_to_resolution(vamp.digit.resolution())
    # traj4 = simple4.path.numpy()



    # now combine the trajectories

    final_traj = np.concatenate((traj1, traj2, traj3, traj4), axis=0)
    attachment_masks = np.concatenate((np.zeros(len(traj1)), np.zeros(len(traj2)), np.ones(len(traj3)), np.zeros(len(traj4))), axis=0)

    # print(final_traj.shape)
    # stop

    # final_traj = traj1


    full_waypoints = np.array(final_traj)
    waypoints_no_floating = full_waypoints[:, 6:]
    final_waypoints = np.zeros((len(waypoints_no_floating), 30))
    final_waypoints[:, :6] = waypoints_no_floating[:, :6]
    final_waypoints[:, 9:21] = waypoints_no_floating[:, 6:18]
    final_waypoints[:, 24:30] = waypoints_no_floating[:, 18:24]
    np.savetxt('/src/vamp/shelf_top_to_down.txt', final_waypoints, fmt='%.4f', delimiter = ',')

    print("Final time: ", result1.nanoseconds/1e6 + result2.nanoseconds/1e6 + result3.nanoseconds/1e6 + result4.nanoseconds/1e6)


    eef_poses = get_eef_of_waypoints(final_traj)
    controller_task_space_traj = convert_trajectory_eef_to_controller_format(final_traj, eef_poses)
    np.savetxt('/src/vamp/shelf_top_to_down_controller_format.txt', controller_task_space_traj, fmt='%.4f', delimiter = ',')


    if len(final_traj) > 0:
        # simple.path.interpolate_to_resolution(vamp.digit.resolution())

        # traj = simple.path.numpy()
        eef_poses = get_eef_of_waypoints(final_traj)

        # find norm between positions of both hands for each waypoint
        hand_distances = np.linalg.norm(eef_poses[:, 0, :3, 3] - eef_poses[:, 1, :3, 3], axis=1)
        # print("min hand distances:", np.min(hand_distances[attachment_masks==1]))
        transport_hand_distances = hand_distances[attachment_masks==1]
        # plot the transport_hand_distances
        plt.figure()
        plt.plot(transport_hand_distances)
        plt.title("Distance between hands during transport")
        plt.xlabel("Waypoint index")
        plt.ylabel("Distance (m)")
        plt.savefig("/src/vamp/hand_distances.png")


        # for viz flatten the first two dimensions so it's just a list of eef poses        
        viz.render_eefs(eef_poses.reshape(-1, 4, 4))
        viz.animate(final_traj, np.arange(0, len(final_traj), dtype=np.float64) / 10, attachment_spheres=attach_sphere_w_r, attachment_masks=attachment_masks, loop=True)


        while True:
            pass
