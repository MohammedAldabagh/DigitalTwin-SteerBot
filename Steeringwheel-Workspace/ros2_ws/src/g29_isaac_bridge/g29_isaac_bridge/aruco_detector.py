import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.minMarkerPerimeterRate = 0.01
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 53
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params)

        self.sub = self.create_subscription(
            Image, '/piper_camera/rgb', self.image_callback, 10)

        self.pub_visible = self.create_publisher(
            Bool, '/wheel/visible', 10)

        self.get_logger().info('ArUco detector started')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        visible_msg = Bool()
        if ids is not None and 0 in ids:
            visible_msg.data = True
            self.get_logger().info('Wheel visible')
        else:
            visible_msg.data = False
            self.get_logger().warn('Wheel NOT visible')
        self.pub_visible.publish(visible_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
