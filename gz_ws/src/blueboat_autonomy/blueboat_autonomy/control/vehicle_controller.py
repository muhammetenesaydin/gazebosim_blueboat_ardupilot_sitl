from geometry_msgs.msg import Twist
from rclpy.node import Node

class VehicleController:
    """
    Interfaces with the vehicle's actuators via ArduPilot.
    Publishes Twist messages which are converted to RC overrides by the interface node.
    """
    def __init__(self, node: Node):
        self.node = node
        
        # Publisher for ArduPilot (via ros2_blueboat_interface)
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/blueboat/cmd_vel',
            10
        )

    def send_command(self, linear_vel, angular_vel):
        """
        Sends linear and angular velocity commands.
        """
        msg = Twist()
        # Map linear_vel (0-1) to forward speed (approx PWM mapping in interface)
        # Interface maps linear.x to channel 5 (Forward)
        # Interface maps angular.z to channel 4 (Yaw)
        
        msg.linear.x = float(linear_vel)
        msg.angular.z = float(angular_vel)
        
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.send_command(0.0, 0.0)
