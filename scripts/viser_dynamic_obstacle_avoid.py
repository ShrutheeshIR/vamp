from typing import Optional, List
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
import numpy as np
import vamp
import json

from sensor_msgs.msg import JointState


rack_2 = [
0.01738501, -0.01096749, -0.47006118, -0.01586935, -0.02395158, -0.00807717  ,                                                                                                               
  0.4024402 ,  0.00743171, -0.22230545, -0.8472173 , -0.01932546,  0.8947249 ,                                                                                                                 
 -0.5300461 ,  0.04379871,  0.09859389,  0.35082862,  0.08830395,  0.13806033,                                                                                                                 
 -0.35991108, -0.00484328,  0.21751796,  0.8449603 ,  0.02007664, -0.9019995 ,                                                                                                                 
  0.41773874,  0.03763357,  0.11142297, -0.31057477, -0.08879709, -0.08477921,
]
box_top_shelf_pickup = [
    1.11889839e-02, -1.99530125e-02,  5.81884384e-03, -3.29416106e-03, -1.42211439e-02, -5.99248195e-03,
    3.84082675e-01,  5.58406231e-04, 2.98647016e-01,  3.22717577e-01, -3.98048153e-03, -3.00990015e-01, 1.07899308e-02,  1.02880923e-02,  
    3.03654131e-02,  1.43816188e-01, -3.00536864e-04, -5.22131145e-01,
     -3.72209966e-01, -3.30008916e-03, -2.98724055e-01, -3.18776041e-01,  8.88650957e-03,  2.83709198e-01, -1.18316561e-01,  6.75600534e-03,  
    4.73398268e-02, -1.23632416e-01, 2.74537895e-02,  4.99689251e-01
]


class ConstrainedVampPubSub(Node):
    """ROS2 node for constrained planner trajectory planning and replanning using VAMP."""

    def setup_planner_and_constraints(self, **kwargs) -> None:
        """Initialize the VAMP planner and configure constraints."""


        polygon_points = [
            0.005, -0.01, 0.005, 0.03, -0.05, 0.03, -0.05, -0.01
        ]

        bimanual_limit_lower_bound = [
            -0.001, -0.001, -0.001, -0.1, -0.1, -10.1
        ]
        bimanual_limit_upper_bound = [
            0.001, 0.001, 0.001, 0.1, 0.1, 10.1
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

        bimanual_transform = [0.1, 0.99, 0.00000, 0.04000, 0.01462, -0.03530, -0.36356];




        (self.vamp_module, self.planner_func, self.plan_settings, self.simp_settings) = (
            vamp.configure_robot_and_planner_with_kwargs("digit", "crrtc", **kwargs)
        )

        bimanual_constraint = self.vamp_module.BimanualTaskSpaceConstraint(
            bimanual_transform,
            bimanual_limit_lower_bound,
            bimanual_limit_upper_bound
        )
        com_constraint = self.vamp_module.CoMTaskSpaceConstraint(
            polygon_points,
        )


        feet_tsr_constraint = self.vamp_module.FeetTaskSpaceConstraint(
            eef_transforms_ref_frame_w_world,
            eef_transforms,
            tsr_lower_bound,
            tsr_upper_bound
        )

        closed_link_constraint = self.vamp_module.ClosedLinkConstraint()


        self.constraints = self.vamp_module.Composable_F_C_CL_B(feet_tsr_constraint, com_constraint, closed_link_constraint, bimanual_constraint)




        self.sampler = self.vamp_module.halton()

        self.goals = [
            box_top_shelf_pickup,
            rack_2
        ]
        self.plan_settings.max_iterations = 1000
        self.plan_settings.range = 0.75
        self.plan_settings.dynamic_domain = False
        self.plan_settings.insert_all_to_tree = True
        self.plan_settings.descend_rate = 1.0;
        self.plan_settings.radius = 10.0;
        self.plan_settings.num_projection_iterations = 10;



        self.current_goal = 1

        tf = np.identity(4)
        self.attachment = vamp.Attachment(tf)


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



        # Add a single sphere to the attachment - spheres are added in the attachment's local frame
        self.attachment.add_spheres(
            [
                vamp.Sphere(pos, 0.04) for pos in attach_spheres
            ]
        )


    def plan(self, start_config, goal_config):
        # both are given in the format that vamp likes so no worries here
        
        t1 = time.perf_counter_ns()
        env = vamp.Environment()
        env.add_sphere(vamp.Sphere(self.obstacle_pose_and_dim[:3], self.obstacle_pose_and_dim[-1]))
        env.attach(self.attachment, 0)

        if self.constraints.distanceToConstraint(np.array(start_config)) > 1e-3:
            start_config = self.constraints.projectConfiguration(np.array(start_config), 1, 0.5, 1.0, 10, False).tolist()

        # now first validate if existing self.trajectory is itself valid
        # we do that by connecting start_config to the currently executing config in the trajectory, and check if all lines are valid. If not, we trigger a replan. This is a bit hacky but it works for now.
        curr_waypoint_index = self.current_time / self.waypoint_dt
        all_waypoints_valid = True
        if self.trajectory is not None and self.trajectory.shape[0] > 0 and (np.linalg.norm(self.trajectory[-1] - goal_config) < 1e-3):  # Only check if we have a trajectory and we're not already at the goal
            for i in range(int(curr_waypoint_index) - 1, self.trajectory.shape[0] - 2):
                if i == int(curr_waypoint_index) - 1:
                    start_config_to_check = start_config
                else:
                    start_config_to_check = self.trajectory[i]
                if (self.vamp_module.validate_motion(start_config_to_check, self.trajectory[i+1], env) == False):
                    self.get_logger().warn("Failed to validate motion from start_config_to_check to trajectory waypoint %d" % (i+1))
                    all_waypoints_valid = False
                    break
            if all_waypoints_valid:
                # print("No change, computed in %.2f ms" % ((time.perf_counter_ns() - t1) / 1e6))
                return
        else:
            if self.trajectory is not None and self.trajectory.shape[0] > 0:
                print("end traj and goal are different : ", self.trajectory[-1], goal_config, np.linalg.norm(self.trajectory[-1] - goal_config))


        result = self.planner_func(start_config, goal_config, env, self.plan_settings, self.constraints, self.sampler)
        if (result.solved == False):
            self.get_logger().warn("Failed to find a solution! start_config=%s, goal_config=%s, cur_obstacle=%s" % (start_config, goal_config, self.obstacle_pose_and_dim))
            self.trajectory = np.array([start_config])
            return
        
        if (len(result.path) > 3):
            simple = self.vamp_module.simplify_with_constraints(result.path, env, self.constraints, self.simp_settings, self.sampler)
        else:
            simple = result           
        # simple = self.vamp_module.simplify(result.path, env, self.simp_settings, self.sampler)
        simple.path.interpolate_to_resolution(int(vamp.panda.resolution() / 4))
        self.trajectory = simple.path.numpy()
        self.current_time = 0.0
        self.trajectory_finished = False
        print("Planned trajectory with %d waypoints in %.2f mseconds" % (self.trajectory.shape[0], (time.perf_counter_ns() - t1) / 1e6))




    def __init__(self):
        super().__init__('humanoid_vamp_pubsub_py')

        # Parameters
        self.declare_parameter('publish_rate_hz', 100.0)  # Publishing frequency in Hz
        self.declare_parameter('waypoint_dt', 0.1)  # Time between waypoints (in the txt file) in seconds (we can reduce this to be smaller than 1 to increase the roll out speed.)
        self.declare_parameter('planner_rate_hz', 10.0)  # Publishing frequency in Hz


        self.setup_planner_and_constraints()
        self.current_waypoint = self.goals[0]
        
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.waypoint_dt = self.get_parameter('waypoint_dt').get_parameter_value().double_value
        self.planner_rate_hz = self.get_parameter('planner_rate_hz').get_parameter_value().double_value

        self.trajectory: Optional[np.ndarray] = None
        self.is_playing = True
        self.current_time = 0.0
        self.trajectory_finished = False
        
        # Obstacle pose and dimensions (8-D array: x, y, z, qw, qx, qy, qz, scale or similar)
        self.obstacle_pose_and_dim: np.ndarray = np.zeros(8)

        self.publisher = self.create_publisher(Float32MultiArray, 'planner_waypoint', 10)
        self.ee_trajectory_publisher = self.create_publisher(Float32MultiArray, 'ee_trajectory', 10)
        self.subscription = self.create_subscription(Int32, 'get_next_waypoint', self.on_get_next_waypoint, 10)
        self.obstacle_subscription = self.create_subscription(Float32MultiArray, '/obstacle_pose_and_dim', self.on_obstacle_pose_and_dim, 10)
        
        # Create timer for high-frequency publishing
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.planner_timer = self.create_timer(1.0 / self.planner_rate_hz, self.planner_callback)


    def on_get_next_waypoint(self, msg: Int32) -> None:
        """Callback to start trajectory playback."""
        if msg.data != 1:
            return

        if self.trajectory is None or self.trajectory.shape[0] == 0:
            self.get_logger().warn("No trajectory loaded; cannot start publishing.")
            return

        if not self.is_playing:
            self.get_logger().info("Starting trajectory playback.")
            self.is_playing = True
            self.current_time = 0.0
            self.trajectory_finished = False

    def on_obstacle_pose_and_dim(self, msg: Float32MultiArray) -> None:
        """Callback to receive obstacle pose and dimensions."""
        if len(msg.data) != 8:
            self.get_logger().warn(f"Expected 8 values for obstacle_pose_and_dim, got {len(msg.data)}")
            return
        
        self.obstacle_pose_and_dim = np.array(msg.data, dtype=np.float32)
        # self.get_logger().info(f"Updated obstacle pose and dimensions: {self.obstacle_pose_and_dim}")

    def planner_callback(self):
        # get the current waypoint and plan to the next one
        start_config = self.current_waypoint
        # start_config = self.goals[self.current_goal - 1]
        goal_config = self.goals[self.current_goal]
        self.plan(start_config, goal_config)

        # Publish end-effector trajectory
        if self.trajectory is not None and self.trajectory.shape[0] > 0:
            ee_poses = []
            for config in self.trajectory:
                eefks = self.vamp_module.eefk(config)
                ee_poses.extend(eefks[0][:3, :4].flatten().tolist())
                ee_poses.extend(eefks[1][:3, :4].flatten().tolist())


            
            ee_msg = Float32MultiArray()
            ee_msg.data = [float(v) for v in ee_poses]
            self.ee_trajectory_publisher.publish(ee_msg)

    def timer_callback(self):
        if not self.is_playing or self.trajectory is None or self.trajectory.shape[0] == 0:
            return

        # Calculate which waypoint we're at
        total_duration = (self.trajectory.shape[0] - 1) * self.waypoint_dt
        
        if self.current_time >= total_duration:
            # We've reached the end - keep publishing the last waypoint
            if not self.trajectory_finished:
                self.get_logger().info("Reached end of trajectory. Publishing last waypoint continuously.")
                self.trajectory_finished = True
            
            wp = self.trajectory[-1, :].tolist()
            waypoint_num = self.trajectory.shape[0] - 1
        else:
            # Interpolate between waypoints
            waypoint_index = self.current_time / self.waypoint_dt
            idx_low = int(np.floor(waypoint_index))
            idx_high = min(idx_low + 1, self.trajectory.shape[0] - 1)
            alpha = waypoint_index - idx_low  # Interpolation factor [0, 1]
            
            # Linear interpolation
            wp_low = self.trajectory[idx_low, :]
            wp_high = self.trajectory[idx_high, :]
            wp = ((1 - alpha) * wp_low + alpha * wp_high).tolist()
            
            waypoint_num = waypoint_index
            
            # Advance time
            self.current_time += 1.0 / self.publish_rate_hz

        # Publish the waypoint
        out = Float32MultiArray()
        out.data = [float(v) for v in wp]
        #  get the eefk of wp and extend it to out.data as well
        out.data.extend(self.vamp_module.eefk(wp)[0][:3, :4].flatten().tolist())

        self.publisher.publish(out)
        self.current_waypoint = wp

        if(np.linalg.norm(np.array(wp) - np.array(self.goals[self.current_goal])) < 1e-3):
            self.get_logger().info("Reached goal.")
            #  move to next goal, and circle back if reached end
            self.current_goal = (self.current_goal + 1) % len(self.goals)



        # Print waypoint info
        # self.get_logger().info(f"Publishing waypoint {waypoint_num:.2f} at time {self.current_time:.4f}s")

def main(args=None):
    rclpy.init(args=args)
    node = ConstrainedVampPubSub()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
