import rclpy
from rclpy.node import Node
from .perception import ZedCameraAdapter
from .mapping import LocalMapper
from .planning import GlobalPlanner, LocalPlanner
from .control import VehicleController
from .mission import MissionController, MissionState
from nav_msgs.msg import Odometry
import numpy as np

class AutonomyNode(Node):
    def __init__(self):
        super().__init__('autonomy_node')
        
        # Initialize Modules
        self.sensor = ZedCameraAdapter(self)
        self.mapper = LocalMapper()
        self.global_planner = GlobalPlanner()
        self.local_planner = LocalPlanner()
        self.controller = VehicleController(self)
        self.mission = MissionController()
        
        # State variables
        self.current_pose = (0.0, 0.0) # x, y
        self.current_yaw = 0.0
        
        # Subscribe to Odometry (Ground Truth for now, or from EKF)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/blueboat/odometry',
            self.odom_callback,
            10
        )
        
        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz
        
        self.get_logger().info("BlueBoat Autonomy Node Started!")

    def odom_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        # 1. Perception
        depth_img = self.sensor.get_depth_image()
        rgb_img = self.sensor.get_rgb_image()
        
        # 2. Mapping
        if depth_img is not None:
            self.mapper.update_map(depth_img)
        
        # 3. Mission Logic
        current_wp = self.global_planner.get_current_waypoint()
        wp_reached = self.global_planner.check_waypoint_reached(*self.current_pose)
        
        if wp_reached:
            self.get_logger().info(f"Waypoint reached! Next: {self.global_planner.get_current_waypoint()}")
            
        # Check for obstacles in map (simplified check)
        grid = self.mapper.get_grid()
        obstacle_detected = np.any(grid > 128) # Very naive check, should check path
        
        # Update Mission State
        state = self.mission.update(wp_reached, obstacle_detected, False) # Target detection TODO
        
        # 4. Planning & Control
        if current_wp:
            # Transform global waypoint to local frame
            dx = current_wp[0] - self.current_pose[0]
            dy = current_wp[1] - self.current_pose[1]
            
            # Rotate to robot frame
            local_x = dx * np.cos(-self.current_yaw) - dy * np.sin(-self.current_yaw)
            local_y = dx * np.sin(-self.current_yaw) + dy * np.cos(-self.current_yaw)
            
            # Local Planner
            linear, angular = self.local_planner.compute_velocity_command(grid, (local_x, local_y))
            
            # Send Command
            self.controller.send_command(linear, angular)
        else:
            # Mission Complete
            self.controller.stop()
            self.get_logger().info("Mission Completed!")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
