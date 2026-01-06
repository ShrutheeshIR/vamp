import time
import viser
import numpy as np
from viser.extras import ViserUrdf
from viser import transforms as tf
# import numpy as mp
# import motion_planner as mp
import yourdfpy 
# import ik_solver
import pyroki as pk
import pyperclip
import jaxlie
import os

import jax.numpy as jnp
jnp.set_printoptions(suppress=True, precision = 3)

def add_sphere():
    return None

def add_cuboids(server, filename):
    cuboids_color = (12, 89, 178)
    cuboids = np.loadtxt(filename,  delimiter = ',')
    print(cuboids)
    for i in range(len(cuboids)):
        server.scene.add_box(
            name = f"/cuboid_{i}",
            dimensions = cuboids[i, 3:] * 2.0,
            color = cuboids_color,
            position = cuboids[i, :3] + [0.1, 1.0, 0.1],
        )


def add_robot_cage(server):
    """
    Adds the lab vention cage to the environment.

    Args:
        server (ViserServer): ViserServer instance

    Returns:
        return_type: None.
    """
    cage_color = (12, 89, 178)
    table = server.scene.add_box(  
        name="/table",  
        dimensions=(1.08, 1.9, 0.09),  
        color=cage_color,  
        position=(0.38, 0.0, -0.045)  
    )
    beam1 = server.scene.add_box(   
        name="/beam1",  
        dimensions=(0.09, 1.8, 0.05),  
        color=cage_color,  
        position=(-0.115, 0.0, 1.775)  
    )
    beam2 = server.scene.add_box(  
        name="/beam2",  
        dimensions=(0.09, 0.05, 1.8),    
        color=cage_color,  
        position=(-0.115, 0.925, 0.9)  
    )
    beam3 = server.scene.add_box(  
        name="/beam3",  
        dimensions=(0.09, 0.05, 1.8),    
        color=cage_color,  
        position=(-0.115, -0.925, 0.9)  
    )
    beam4 = server.scene.add_box(  
        name="/beam4",  
        dimensions=(0.09, 0.05, 1.8),    
        color=cage_color,  
        position=(0.875, 0.925, 0.9)  
    )
    beam5 = server.scene.add_box(  
        name="/beam5",  
        dimensions=(0.09, 0.05, 1.8),   
        color=cage_color,  
        position=(0.875, -0.925, 0.9)  
    )
    beam6 = server.scene.add_box(   
        name="/beam6",  
        dimensions=(0.09, 1.8, 0.05),   
        color=cage_color,  
        position=(0.875, 0.0, 1.775)  
    )
    beam7 = server.scene.add_box(   
        name="/beam7",  
        dimensions=(0.09, 1.8, 0.05),    
        color=cage_color,  
        position=(0.875, 0.0, 1.075)  
    )
    
    beam8 = server.scene.add_box(   
        name="/beam8",  
        dimensions=(0.9, 0.05, 0.09),    
        color=cage_color,  
        position=(0.38, 0.925, 1.075)  
    )
    beam9 = server.scene.add_box(   
        name="/beam9",  
        dimensions=(0.9, 0.05, 0.09),    
        color=cage_color,  
        position=(0.38, -0.925, 1.075)  
    )
    beam10 = server.scene.add_box(   
        name="/beam10",  
        dimensions=(0.9, 0.05, 0.09),    
        color=cage_color,  
        position=(0.38, 0.925, 1.755)  
    )
    beam11 = server.scene.add_box(   
        name="/beam11",  
        dimensions=(0.9, 0.05, 0.09),    
        color=cage_color,  
        position=(0.38, -0.925, 1.755)  
    )
    

def add_static_pc(server = None, pc = None, colors = None):
    """
    Adds a static point cloud to the scene with respect to the robots origin

    Args:
        server (ViserServer): ViserServer instance
        pc (numpy.array): A 2D numpy array (shape: (N,3)) with N points representing a point cloud
        colors (numpy.array): A 2D numpy array (shape: (N,3)) with colors for each point

    Returns:
        return_type: None
    """
    server.scene.add_point_cloud(
        name="/jug_pc",
        points=pc,
        colors=colors,
        point_size=0.003,
    )
    
def add_trajectory(server, waypoints, panda):
    """
    Adds a slider to step through waypoints of a trajectory also allows for auto step through
    using play/pause button

    Args:
        server (ViserServer): ViserServer instance
        waypoints (numpy.array): A 2D numpy array (shape: (N,7)) with N waypoints of joint poses 
        panda (ViserUrdf): ViserUrdf instance of the robot

    Returns:
        return_type: None.
    """
    traj_slider = server.gui.add_slider(  
        "Current Waypoint",   
        min=0,   
        max=len(waypoints)-1,   
        step=1,   
        initial_value=0  
    )
    @traj_slider.on_update  
    def update_robot_pose(event):  
        waypoint_idx = int(event.target.value)  
        joint_config = waypoints[waypoint_idx]  
        panda.update_cfg(joint_config)  


def create_viser_stick_gripper(server, name="/stick_gripper", scale=1.0, line_width=5.0):  
    """  
    Creates a stick figure representation of a robot gripper using line segments.  
  
    Args:  
        server (ViserServer): ViserServer instance for creating scene objects  
        name (str): Scene name for the gripper. 
        scale (float): Scale factor for gripper dimensions.
        line_width (float): Width of the line segments in space.
  
    Returns:  
        LineSegmentsHandle: Handle for manipulating the gripper's position,   
            orientation, and other properties in the scene  
    """
    finger_offset_x = 0.041 * scale  
    finger_height = (0.11217 - 0.0659999996) * scale  
    finger_base_z = 0.0659999996 * scale  
    stick_height = finger_base_z  
    bar_length = 0.082 * scale  
    
    line_segments = []  
    colors = []   
    
    # Gripper from origin to base of claw  
    line_segments.append([[0, 0, 0], [0, 0, stick_height]])  
    colors.append([[0, 0, 255], [0, 0, 255]])  
 
    line_segments.append([  
        [finger_offset_x, 0, finger_base_z],   
        [finger_offset_x, 0, finger_base_z + finger_height]  
    ])  
    colors.append([[0, 0, 255], [0, 0, 255]])  
     
    line_segments.append([  
        [-finger_offset_x, 0, finger_base_z],   
        [-finger_offset_x, 0, finger_base_z + finger_height]  
    ])  
    colors.append([[0, 0, 255], [0, 0, 255]])  
     
    line_segments.append([  
        [-bar_length / 2, 0, finger_base_z],   
        [bar_length / 2, 0, finger_base_z]  
    ])  
    colors.append([[0, 0, 255], [0, 0, 255]])  
    
    points = np.array(line_segments, dtype=np.float32)  
    colors = np.array(colors, dtype=np.uint8)  
     
    gripper_handle = server.scene.add_line_segments(  
        name=name,  
        points=points,  
        colors=colors,  
        line_width=line_width  
    )  
      
    return gripper_handle

def plan(server, panda, start, goal):
     plan = server.gui.add_button("Plan from start to goal")  

     @plan.on_click
     def plan(event):
        waypoints = mp.generate_trajectory(start, goal)
        print(f'solved motion:{waypoints}')
        # progress_message.remove()
        add_trajectory(server, waypoints, panda)



# def control_ee(server, panda):
#     """
#     Allows user to input a specific ee pose and then plans and moves robot to that pose

#     Args:
#         server (ViserServer): ViserServer instance

#     Returns:
#         return_type: None.
#     """
    
#     insert_grasp = server.gui.add_button("Insert Grasp")  
#     @insert_grasp.on_click
#     def ik_and_plan(event):
#         stick_gripper = create_viser_stick_gripper(server)
#         insert_grasp.remove()
#         cancel_plan = server.gui.add_button("Cancel Grasp Planning")
#         solve_ik = server.gui.add_button("Solve IK")
#         plan = server.gui.add_button("Plan motion")
#         plan.visible = False
        
#         xyz_input = server.gui.add_vector3(  
#             "Position (XYZ) meters",  
#             initial_value=(0.47, 0.0, 0.45),  
#             min=(-10.0, -10.0, -10.0),  
#             max=(10.0, 10.0, 10.0),  
#             step=0.01  
#         )
#         rpy_input = server.gui.add_vector3(  
#             "Rotation (RPY) radians",  
#             initial_value=(3.14, 0.0, 1.59),  
#             min=(-3.14159, -3.14159, -3.14159),  
#             max=(3.14159, 3.14159, 3.14159),  
#             step=0.01  
#         )
#         position = xyz_input.value  
#         rpy = rpy_input.value
#         wxyz = tf.SO3.from_rpy_radians(rpy[0], rpy[1], rpy[2]).wxyz  
#         stick_gripper.position = position  
#         stick_gripper.wxyz = wxyz
        
#         panda_shadow = None
        
#         @xyz_input.on_update  
#         def update_gripper_pose(event):  
#             position = xyz_input.value  
#             rpy = rpy_input.value
#             wxyz = tf.SO3.from_rpy_radians(rpy[0], rpy[1], rpy[2]).wxyz  
#             stick_gripper.position = position  
#             stick_gripper.wxyz = wxyz
        
#         @rpy_input.on_update    
#         def update_gripper_rotation(event):  
#             update_gripper_pose(event)
        
#         @solve_ik.on_click  
#         def solve_ik(event): 
#             cancel_plan.visible = False  
#             solve_ik.visible = False    
#             xyz_input.visible = False  
#             rpy_input.visible = False
#             progress_message = server.gui.add_markdown("**Solving IK...**")
#             nonlocal panda_shadow
#             position = stick_gripper.position  
#             orientation = stick_gripper.wxyz  
            
#             config = ik_solver.compute_ik([position, orientation])
#             print(config)
#             urdf = yourdfpy.URDF.load("assets/panda/bipanda_spherized.urdf")
#             panda_shadow = ViserUrdf(  
#                 server,  
#                 urdf,
#                 load_meshes=True,
#                 load_collision_meshes=False,
#                 root_node_name="/panda_shadow",
#                 mesh_color_override=(0, 255, 0, 0.8)
#             )
#             panda_shadow.update_cfg(np.array(config))
#             progress_message.remove()
#             cancel_plan.visible = True  
#             solve_ik.visible = True    
#             plan.visible = True
#             xyz_input.visible = True  
#             rpy_input.visible = True
#             @plan.on_click  
#             def move(event):
#                 cancel_plan.visible = False  
#                 plan.visible = False
#                 solve_ik.visible = False    
#                 xyz_input.visible = False  
#                 rpy_input.visible = False
#                 progress_message = server.gui.add_markdown("**Planning Trajectory...**")
#                 print(f'start q:{panda._urdf.cfg}')
#                 print(f'end q:{config}')
#                 waypoints = mp.generate_trajectory(panda._urdf.cfg, config)
#                 print(f'solved motion:{waypoints}')
#                 progress_message.remove()
#                 add_trajectory(server, waypoints, panda)
                
            
#         @cancel_plan.on_click
#         def cancel_grasp(event):  
#             cancel_plan.remove()  
#             plan.remove()
#             solve_ik.remove()  
#             xyz_input.remove()  
#             rpy_input.remove()
#             stick_gripper.remove()
#             if panda_shadow is not None:
#                 panda_shadow.remove()
#             # Recreate the original "Insert Grasp" button  
#             nonlocal insert_grasp  
#             insert_grasp = server.gui.add_button("Insert Grasp")  
#             insert_grasp.on_click(ik_and_plan)

urdf = yourdfpy.URDF.load("/vamp/resources/cricket/cricket_spherized.urdf")
# urdf = yourdfpy.URDF.load("assets/kuka_iiwa/spherized_kuka/kuka_spherized_180.urdf")
robot = pk.Robot.from_urdf(urdf)
def fk(config):
    eefk = robot.forward_kinematics(np.array(config))
    link_poses_se3 = jaxlie.SE3(eefk)  
    
    # Get 4x4 transformation matrices  
    transformation_matrices = link_poses_se3.as_matrix()  # Shape: *batch, link_count, 4, 4
    return transformation_matrices



def add_joint_sliders(server, panda, initial_values = None):
    
    joint_names = panda.get_actuated_joint_names()  
    joint_limits = panda.get_actuated_joint_limits()

    copy_joint_config = server.gui.add_button("Copy Joint Config to clipboard")
    
    joint_sliders = {}  
    for jidx, joint_name in enumerate(joint_names):  
        min_val, max_val = joint_limits[joint_name]   
        if min_val is None:  
            min_val = -3.14159  
        if max_val is None:  
            max_val = 3.14159  
        
        slider = server.gui.add_slider(  
            label=f"Joint {joint_name}",  
            min=min_val,  
            max=max_val,  
            step=0.01,  
            initial_value=initial_values[jidx]
        )  
        joint_sliders[joint_name] = slider
        
    def update_robot_joints():  
        joint_config = []  
        for joint_name in joint_names:  
            joint_config.append(joint_sliders[joint_name].value)   
        panda.update_cfg(np.array(joint_config))  
        
        # Print ee pose to ui using fk on joint pose
        links_poses = fk(joint_config)
        # position_text = server.gui.add_text(  
        #     label="Position",   
        #     initial_value=f"x: {links_poses[9][4]:.3f}, y: {links_poses[9][5]:.3f}, z: {links_poses[9][6]:.3f}"  
        # )
        # quat_text = server.gui.add_text(  
        #     label="Quat",   
        #     initial_value=f"w: {links_poses[9][0]:.3f}, x: {links_poses[9][1]:.3f}, y: {links_poses[9][2]:.3f}, z: {links_poses[9][3]:.3f}"  
        # )
        

        # print(f'Pose : ', links_poses.shape)
        # print('----')
        # print(f'Position xyz: {np.array(links_poses[9][4:])}')
        # print(np.array(links_poses[9][:3, :3]))
        # print(jaxlie.SO3(np.array(links_poses[9][:3, :3])).log())
        # print(jaxlie.SO3.from_matrix(links_poses[..., 9, :3, :3]).log())
        # print(jaxlie.SO3.from_matrix(links_poses[..., 9, :3, :3]).as_matrix())

        # server.add_frame(
        #     name = '/ee_frame',
        #     axes_length = 0.2,
        #     axes_radius = 0.01,
        #     wxyz = np.array(links_poses[9][:4]),
        #     position = np.array(links_poses[9][4:])
        # )

    @copy_joint_config.on_click
    def copy_func(event):
        progress_message = server.gui.add_markdown(",".join([str(c) for c in panda._urdf.cfg]))
        
        # pyperclip.copy(",".join([str(c) for c in panda._urdf.cfg]))

        
        # link_poses = urdf.forward_kinematics_joint(joint_config)  
        # end_effector_pose = urdf._urdf.get_transform('panda_hand')
        # print(end_effector_pose)
    
    for slider in joint_sliders.values():  
        slider.on_update(lambda event: update_robot_joints())
        


# start = [float(x) for x in "0.930205 0.966287 0.194365 -1.51657 -0.6965 3.8223 -0.959755 1.14244 0.93196 -0.00581666 -1.49359 -0.609867 0.687591 -0.73099".split(' ')]
# goal = [float(x) for x in "-0.67113 1.66257 0.0235852 -1.51145 1.2212 1.35781 -0.959755 -0.375588 1.3998 -0.174771 -1.43568 -1.51361 1.9924 -0.73099".split(' ')]
# print(start, goal)
# print(start1[:7], start1[7:])
# start1 = 0.0,0.0,0.0,-1.57,1.57,1.57,0.0,0.0,0.0,0.0,-1.57,-1.57,1.57,1.57
# start1 = [0.917903, 1.00429, 0.146204, -1.42557, -0.607875, 3.68894, -0.958184, 1.02408, 0.942458, -0.0666186, -1.64013, -0.60696, 0.768222, -0.730373]

# start1 = [-0.273508 -0.386581 0.241059 -2.99334 -0.298719 2.36136 2.25229 -1.13086 0.92905 -0.235822 -1.49836 -0.259057 2.59989 1.02771]
start1 = [-0.816, -0.203, 0.716, -0.961, 0.770, 2.055, -0.360,]
# start1 = [-0.64309,  1.91561, -1.79683,  1.29454, -0.02383, -0.87697, -1.70416]
# print(fk(start1)[..., 9, :, :])
# Problem specification: a list of sphere centers
# sphere_positions = [
#     # [0.55, 0, 0.25],
#     [0.55, 0, 0.50],
#     [0.35, 0.35, 0.25],
#     [0, 0.55, 0.25],
#     [-0.55, 0, 0.25],
#     [-0.35, -0.35, 0.25],
#     [0, -0.55, 0.25],
#     [0.35, -0.35, 0.25],
#     [0.35, 0.35, 0.8],
#     [0, 0.55, 0.8],
#     [-0.35, 0.35, 0.8],
#     [-0.55, 0, 0.8],
#     [-0.35, -0.35, 0.8],
#     [0, -0.55, 0.8],
#     [0.35, -0.35, 0.8],
#     ]


def add_spheres(server, sphere_positions = None):
    sphere_handles = [None] * len(sphere_positions)
    for i, sphere in enumerate(sphere_positions):
        print(tuple(sphere))
        sphere_handles[i] = server.scene.add_icosphere(  
            name=f"my_sphere_{i}",
            radius=sphere[3],  
            position=tuple(sphere[:3]),  
        )
    return sphere_handles

def add_constraint_plane(server):
    box_handle = server.scene.add_box(  
        name="my_cuboid",  
        dimensions=(0.02, 5.0, 5.0),  # (width, height, depth)  
        position=(0.54, 0.0, 0.0),  
        color=(0, 255, 0),  # Red color  
        opacity=0.5
    )    

def main():
    server = viser.ViserServer()
    
    floor_grid = server.scene.add_grid(  
        name="/floor_grid",  
        width=10.0,  
        height=10.0,  
        plane="xy",
        position=(0.0, 0.0, 0.0), 
        cell_color=(200, 200, 200),  
        section_color=(140, 140, 140)  
    )
    
    urdf = yourdfpy.URDF.load("/vamp/resources/cricket/cricket_spherized.urdf")
    panda = ViserUrdf(  
        server,  
        urdf,
        load_meshes=True,
        load_collision_meshes=True,
        root_node_name="/panda"  
    )

    panda.update_cfg(start1)
    # add_robot_cage(server)
    # add_cuboids(server, 'environments/stl/maze_cuboids.txt')

    prev_mtime = 0.0
    prev_sphere_mtime = 0.0


    traj_path = "/vamp/trajectory.txt"
    sphere_path = "/vamp/spheres.txt"

    # traj_path = "/src/fruit-ninja/src/geoplanner/notebooks/trajectory.txt"

    # pc = np.load("/src/vamp/myfork/vamp/environments/pcs/maze.npy")
    # colors = np.zeros((pc.shape[0], 3), dtype=np.uint8)
    # colors[:, 0] = 255
    # add_static_pc(server, pc, colors)

    add_joint_sliders(server, panda, start1)
    # add_constraint_plane(server)
    sphere_handles = None

    # goal = [0.246, 0.670, 0.151]
    # start = [0.441, 0.738, 0.150]

    # server.scene.add_icosphere(  
    #         name=f"start",
    #         radius=0.025,  
    #         position=tuple(start),
    #         color = (0,0,0)
    #     )
    # server.scene.add_icosphere(  
    #         name=f"goal",
    #         radius=0.025,  
    #         position=tuple(goal),  
    #         color = (252,255,255)
    #     )

    while True:
        current_mtime = os.path.getmtime(traj_path)
        current_sphere_mtime = os.path.getmtime(sphere_path)

        if current_mtime != prev_mtime or current_sphere_mtime != prev_sphere_mtime:
            time.sleep(1.0)
            prev_mtime = current_mtime
            prev_sphere_mtime = current_sphere_mtime
            print("File has been modified, reloading trajectory...")
            with open(traj_path, 'r') as f:
                lines = f.readlines()
            if len(lines) == 0:
                continue
            full_waypoints = []
            for line in lines:
                waypoint = [float(x) for x in line.split(',')]
                full_waypoints.append(waypoint)
            # full_waypoints = [start, goal]
            # print(full_waypoints)
            full_waypoints = np.array(full_waypoints)
            # fks = fk(full_waypoints)
            # full_waypoints = np.linspace(start, goal, 50)
            # print(full_waypoints.shape)
            add_trajectory(server, np.array(full_waypoints), panda)

            with open(sphere_path, 'r') as f:
                lines = f.readlines()
            if sphere_handles is not None:
                for handle in sphere_handles:
                    handle.remove()
            spheres = []
            for line in lines:
                position = [float(x) for x in line.split(',')]
                spheres.append(position)
            sphere_handles = add_spheres(server, spheres)

            time.sleep(1.0)


    while True:
        time.sleep(1.0)

if __name__ == "__main__":
    main()
