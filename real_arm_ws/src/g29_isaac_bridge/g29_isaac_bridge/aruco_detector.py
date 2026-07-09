import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from scipy.spatial.transform import Rotation

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.params = cv2.aruco.DetectorParameters()
        self.params.minMarkerPerimeterRate = 0.01
        self.params.maxMarkerPerimeterRate = 4.0
        self.params.adaptiveThreshWinSizeMin = 3
        self.params.adaptiveThreshWinSizeMax = 53
        self.params.adaptiveThreshWinSizeStep = 4
        self.params.adaptiveThreshConstant = 7
        self.params.minCornerDistanceRate = 0.01
        self.params.minDistanceToBorder = 1
        self.params.polygonalApproxAccuracyRate = 0.05
        self.params.minOtsuStdDev = 1.0
        self.params.perspectiveRemovePixelPerCell = 8
        self.params.perspectiveRemoveIgnoredMarginPerCell = 0.1
        self.params.maxErroneousBitsInBorderRate = 0.5
        self.params.errorCorrectionRate = 1.0

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.params)
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        self.pub_visible = self.create_publisher(Bool, '/wheel/visible', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.camera_matrix = np.array([
            [614.0,   0.0, 320.0],
            [  0.0, 614.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        self.marker_size = 0.08

        self.obj_points = np.array([
            [-self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2,  self.marker_size/2, 0],
            [ self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)

        self.missed_frames = 0
        self.last_tf = None

        self.get_logger().info('ArUco detector started')

    def try_detect(self, gray):
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None and 0 in ids:
            return corners, ids

        enhanced = self.clahe.apply(gray)
        corners, ids, _ = self.detector.detectMarkers(enhanced)
        if ids is not None and 0 in ids:
            return corners, ids

        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
        sharpened = cv2.filter2D(enhanced, -1, kernel)
        corners, ids, _ = self.detector.detectMarkers(sharpened)
        if ids is not None and 0 in ids:
            return corners, ids

        upscaled = cv2.resize(enhanced, (gray.shape[1]*2, gray.shape[0]*2))
        corners, ids, _ = self.detector.detectMarkers(upscaled)
        if ids is not None and 0 in ids:
            corners = [c / 2.0 for c in corners]
            return corners, ids

        bright = cv2.convertScaleAbs(gray, alpha=1.5, beta=30)
        corners, ids, _ = self.detector.detectMarkers(bright)
        if ids is not None and 0 in ids:
            return corners, ids

        return None, None

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids = self.try_detect(gray)

        visible_msg = Bool()

        if ids is not None and 0 in ids:
            visible_msg.data = True
            self.pub_visible.publish(visible_msg)
            self.missed_frames = 0

            idx = list(ids.flatten()).index(0)

            success, rvec, tvec = cv2.solvePnP(
                self.obj_points, corners[idx],
                self.camera_matrix, self.dist_coeffs)

            tvec = tvec.flatten()

            # First publish in camera_color_optical_frame
            # TF chain: world -> camera_link -> camera_color_frame
            #           -> camera_color_optical_frame -> g29_joint_axis
            t_cam = TransformStamped()
            t_cam.header.stamp = self.get_clock().now().to_msg()
            t_cam.header.frame_id = 'camera_color_optical_frame'
            t_cam.child_frame_id = 'g29_joint_axis_cam'
            t_cam.transform.translation.x = float(tvec[0])
            t_cam.transform.translation.y = float(tvec[1])
            t_cam.transform.translation.z = float(tvec[2])
            t_cam.transform.rotation.x = 0.0
            t_cam.transform.rotation.y = 0.0
            t_cam.transform.rotation.z = 0.0
            t_cam.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_cam)

            # Now look up g29_joint_axis_cam in world frame
            try:
                world_tf = self.tf_buffer.lookup_transform(
                    'world', 'g29_joint_axis_cam',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1))

                # Publish g29_joint_axis in world frame with fixed orientation
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = 'g29_joint_axis'
                t.transform.translation.x = world_tf.transform.translation.x
                t.transform.translation.y = world_tf.transform.translation.y
                t.transform.translation.z = world_tf.transform.translation.z
                # Fixed orientation — wheel tilt never changes
                t.transform.rotation.x =  0.369082
                t.transform.rotation.y = -0.369084
                t.transform.rotation.z = -0.603141
                t.transform.rotation.w =  0.603139
                self.tf_broadcaster.sendTransform(t)
                self.last_tf = t

                self.get_logger().info(
                    f'Detected — world x:{world_tf.transform.translation.x:.3f}m '
                    f'y:{world_tf.transform.translation.y:.3f}m '
                    f'z:{world_tf.transform.translation.z:.3f}m')

            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}')

        else:
            if self.last_tf is not None and self.missed_frames < 10:
                self.missed_frames += 1
                visible_msg.data = True
                self.pub_visible.publish(visible_msg)
                self.last_tf.header.stamp = self.get_clock().now().to_msg()
                self.tf_broadcaster.sendTransform(self.last_tf)
                self.get_logger().warn(
                    f'Using last known position (missed {self.missed_frames} frames)')
            else:
                visible_msg.data = False
                self.pub_visible.publish(visible_msg)
                self.get_logger().warn('Wheel NOT visible')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
