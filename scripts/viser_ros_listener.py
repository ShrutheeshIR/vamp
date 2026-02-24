from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import json
from scipy.spatial.transform import Rotation
from viser_utils import setup_viser_with_robot

def add_constraint_plane(server, position=(0.29276255, -0.55347496, 0.00607783), wxyz=(0, 1, 0, 0)):
    print(position, wxyz)
    box_handle = server.scene.add_box(
        name="my_cuboid",
        dimensions=(10.02, 10.5, 0.06),  # (width, height, depth)
        position=position,
        wxyz = wxyz,
        color=(0, 255, 0),  # Red color
        opacity=0.5,
    )

def add_json_cuboids(server, filename, color=(12, 89, 178)):
    """
    Add cuboids from a JSON file. The JSON should be a list of objects, each with keys:
    'name', 'x', 'y', 'z', 'dx', 'dy', 'dz' and optional 'roll','pitch','yaw'.

    This function reads the JSON, iterates over entries, and adds a box for each item.
    Dimensions are used as provided in the JSON (assumed full extents).
    """
    with open(filename, "r") as f:
        data = json.load(f)

    for i, obj in enumerate(data):
        name = obj.get("name", f"cuboid_{i}")
        x = obj.get("x", 0.0)
        y = obj.get("y", 0.0)
        z = obj.get("z", 0.0)
        dx = obj.get("dx", 0.0)
        dy = obj.get("dy", 0.0)
        dz = obj.get("dz", 0.0)

        # Position and dimensions expected as (x, y, z) and (dx, dy, dz)
        server.scene.add_box(
            name=f"/{name}",
            dimensions=(dx, dy, dz),
            color=color,
            position=(x, y, z),
        )


class ViserWaypointListener(Node):
	def __init__(self) -> None:
		super().__init__("viser_waypoint_listener")

		self.declare_parameter("waypoint_topic", "planner_waypoint")
		self.declare_parameter("frame_pose_topic", "obstacle_pose_and_dim")
		self.declare_parameter("ee_trajectory_topic", "ee_trajectory")
		self.declare_parameter("frame_publish_rate_hz", 10.0)

		waypoint_topic = self.get_parameter("waypoint_topic").get_parameter_value().string_value
		frame_pose_topic = self.get_parameter("frame_pose_topic").get_parameter_value().string_value
		ee_trajectory_topic = self.get_parameter("ee_trajectory_topic").get_parameter_value().string_value
		frame_publish_rate_hz = self.get_parameter("frame_publish_rate_hz").get_parameter_value().double_value

		robot_dir = Path(__file__).parents[1] / "resources" / "digit_description"
		# robot_dir = Path("/src/myfork_cricket/cricket/resources/panda_marker/")
		self.server, self.robot = setup_viser_with_robot(robot_dir, "digit_model_trace_collision_spherized.urdf")

		self.robot_dof = 30
		self.current_waypoint = np.array(
			[-0.88021, 0.53120, -0.20601, -1.61905, 0.11733, 2.14908, 1.19294],
			dtype=np.float64,
		)
		# self.robot.update_cfg(self.current_waypoint.tolist())
		
		# Track waypoint spheres for trajectory visualization
		self.waypoint_sphere_handles = []

		self.frame_pose_pub = self.create_publisher(Float32MultiArray, frame_pose_topic, 10)
		self.waypoint_sub = self.create_subscription(
			Float32MultiArray,
			waypoint_topic,
			self.on_waypoint,
			10,
		)
		self.ee_trajectory_sub = self.create_subscription(
			Float32MultiArray,
			ee_trajectory_topic,
			self.on_ee_trajectory,
			10,
		)

		# self.hardcoded_frame_pose = np.array(
		# 	[0.30, -0.20, 0.50, 0.0, 0.0, 0.0, 1.0],
		# 	dtype=np.float64,
		# )
		self.frame_pub_timer = self.create_timer(
			1.0 / frame_publish_rate_hz,
			self.publish_hardcoded_frame_pose,
		)

		# Create interactive controller and mesh for the sphere obstacle.
		self.sphere_handle = self.server.scene.add_transform_controls(
			"/obstacle", scale=0.2, position=(0.4, 0.3, 0.4)
		)
		self.server.scene.add_icosphere(
            name="/obstacle/mesh",
            radius=0.05,
            position=(0.0, 0.0, 0.0),
            color=(1.0, 0.0, 0.0),
        )

		self.timing_handle = self.server.gui.add_number("Elapsed (ms)", 0.001, disabled=True)

		# Add a box cuboid that will be transformed based on EE position
		self.ee_box_handle = self.server.scene.add_box(
			name="/ee_box",
			dimensions=(0.1, 0.1, 0.36),
			position=(0.0, 0.0, 0.0),
			wxyz=(1.0, 0.0, 0.0, 0.0),
			color=(255, 165, 0),  # Orange color
			opacity=0.7,
		)

		self.get_logger().info(
			f"Viser listener ready. Subscribed to '{waypoint_topic}', publishing frame pose on '{frame_pose_topic}'."
		)

	def on_waypoint(self, msg: Float32MultiArray) -> None:
		waypoint = np.array(msg.data, dtype=np.float64)[:self.robot_dof]
		eefk_data = np.array(msg.data, dtype=np.float64)[self.robot_dof:self.robot_dof+12]

		if waypoint.size != self.robot_dof:
			self.get_logger().warn(
				f"Received waypoint with {waypoint.size} values, expected {self.robot_dof}. Ignoring."
			)
			return

		self.current_waypoint = waypoint
		self.robot.update_cfg(self.current_waypoint.tolist())
		
		# Transform the box using eefk_data (assuming it's a 3x4 transformation matrix flattened as 12 elements)
		if eefk_data.size >= 12:
			# Extract position from the transformation matrix (last column: indices 3, 7, 11)
			position = (float(eefk_data[3]), float(eefk_data[7])  - 0.16, float(eefk_data[11]))  # Adjust Z by subtracting 0.16 to align with the end of the box
			
			# Extract rotation matrix (3x3) and convert to quaternion
			rotation_matrix = np.array([
				[eefk_data[0], eefk_data[1], eefk_data[2]],
				[eefk_data[4], eefk_data[5], eefk_data[6]],
				[eefk_data[8], eefk_data[9], eefk_data[10]]
			])
			
			# Convert rotation matrix to quaternion using scipy
			# scipy returns (x, y, z, w), but viser expects (w, x, y, z)
			quat_xyzw = Rotation.from_matrix(rotation_matrix).as_quat()
			wxyz = (quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2])
			
			# Update box position and orientation
			self.ee_box_handle.position = position
			self.ee_box_handle.wxyz = wxyz

	def on_ee_trajectory(self, msg: Float32MultiArray) -> None:
		"""Receive EE trajectory and visualize as waypoint spheres."""
		# Remove existing waypoint spheres
		for handle in self.waypoint_sphere_handles:
			handle.remove()
		self.waypoint_sphere_handles.clear()
		
		# Parse EE positions (message contains flattened [x1, y1, z1, x2, y2, z2, ...])
		if len(msg.data) % 3 != 0:
			self.get_logger().warn(f"EE trajectory data length {len(msg.data)} is not divisible by 3")
			return
		
		num_waypoints = len(msg.data) // 24 # assuming 2 eefs with a 3x4 matrix each
		for i in range(num_waypoints):
			x = msg.data[24*i + 3]
			y = msg.data[24*i + 7]
			z = msg.data[24*i + 11]
			
			# Add a small sphere at each waypoint position
			handle = self.server.scene.add_icosphere(
				name=f"/trajectory_waypoint_eef_1{i}",
				radius=0.0075,
				position=(x, y, z),
				color=(0.0, 1.0, 0.0),  # Green color for waypoints
			)
			self.waypoint_sphere_handles.append(handle)

			# now for the other eef
			x = msg.data[24*i + 15]
			y = msg.data[24*i + 19]
			z = msg.data[24*i + 23]

			handle = self.server.scene.add_icosphere(
				name=f"/trajectory_waypoint_eef_2{i}",
				radius=0.0075,
				position=(x, y, z),
				color=(0.0, 1.0, 0.0),  #
			)
			self.waypoint_sphere_handles.append(handle)
		

	def publish_hardcoded_frame_pose(self) -> None:
		out = Float32MultiArray()
		wxyz=np.array(self.sphere_handle.wxyz)
		position=np.array(self.sphere_handle.position)
		out.data = [float(v) for v in np.concatenate((position, wxyz, np.array([0.05])))]
		self.frame_pose_pub.publish(out)




def main(args=None) -> None:
	rclpy.init(args=args)
	node = ViserWaypointListener()
	try:
		rclpy.spin(node)
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
