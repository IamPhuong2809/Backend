import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import threading

class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        self.bridge = CvBridge()
        self.camera_info = None
        self.detected_event = threading.Event()

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10
        )

        # Publisher for debug image
        self.image_pub = self.create_publisher(Image, '/aruco/image_markers', 10)

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # ArUco dictionary and detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

    def info_callback(self, msg):
        self.camera_info = msg

    def get_marker_corners_3d(self, marker_length):
        half_len = marker_length / 2
        return np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)

    def image_callback(self, msg):
        if self.camera_info is None or self.detected_event.is_set():
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect markers
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None:
            marker_length = 0.05  # 5cm
            K = np.array(self.camera_info.k).reshape((3, 3))
            D = np.array(self.camera_info.d)
            object_points = self.get_marker_corners_3d(marker_length)

            for i in range(len(ids)):
                image_points = corners[i][0].astype(np.float32)  # 4x2
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, D)
                if not success:
                    continue

                cv2.drawFrameAxes(cv_image, K, D, rvec, tvec, 0.03)

                # TF
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f'aruco_{ids[i][0]}'
                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])

                q = R.from_rotvec(rvec.flatten()).as_quat()
                t.transform.rotation.x = float(q[0])
                t.transform.rotation.y = float(q[1])
                t.transform.rotation.z = float(q[2])
                t.transform.rotation.w = float(q[3])

                self.br.sendTransform(t)
                self.get_logger().info(f"Marker {ids[i][0]}: Pos {tvec.flatten()}")

                # Dừng sau khi phát hiện 1 marker
                # self.detected_event.set()

        # Publish debug image
        img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        img_msg.header = msg.header
        self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()