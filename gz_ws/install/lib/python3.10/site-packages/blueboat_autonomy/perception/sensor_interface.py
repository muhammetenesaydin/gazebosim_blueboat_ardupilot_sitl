from abc import ABC, abstractmethod
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class SensorInterface(ABC):
    """
    Abstract base class for sensor interfaces.
    Follows the Interface Segregation Principle (ISP) from SOLID.
    """
    @abstractmethod
    def get_rgb_image(self):
        pass

    @abstractmethod
    def get_depth_image(self):
        pass

class ZedCameraAdapter(SensorInterface):
    """
    Adapter for the ZED Camera using ROS 2 topics.
    Follows the Adapter Pattern.
    """
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None

        # Subscriptions
        self.rgb_sub = self.node.create_subscription(
            Image,
            '/zed/rgb/image_raw',
            self.rgb_callback,
            10
        )
        self.depth_sub = self.node.create_subscription(
            Image,
            '/zed/depth/image_raw',
            self.depth_callback,
            10
        )
        self.node.get_logger().info("ZED Camera Adapter initialized.")

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert RGB image: {e}")

    def depth_callback(self, msg):
        try:
            # Depth in Gazebo is usually float32 (meters)
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.node.get_logger().error(f"Failed to convert Depth image: {e}")

    def get_rgb_image(self):
        return self.rgb_image

    def get_depth_image(self):
        return self.depth_image
