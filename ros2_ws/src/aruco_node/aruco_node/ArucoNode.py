import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from custom_service.srv import EnableMarker
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetectNode(Node):
    def __init__(self):
        super().__init__('aruco_detect_node')
        self.camera_info = None
        self.result = None
        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        self.image_sub = None
        self.info_sub = None

        # self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        # self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.service = self.create_service(EnableMarker, '/aruco/enable_marker_detection', self.handle_service)
        self.marker_id_pub = self.create_publisher(Int32, '/aruco/marker_id', 10)

    
    def activate_subscribers(self):
        """Kích hoạt subscribers khi cần thiết"""
        if self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image, 
                '/camera/camera/color/image_raw', 
                self.image_callback, 
                10
            )
        if self.info_sub is None:
            self.info_sub = self.create_subscription(
                CameraInfo, 
                '/camera/camera/color/camera_info', 
                self.info_callback, 
                10
            )

    def deactivate_subscribers(self):
        """Hủy subscribers khi không cần thiết"""
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.info_sub is not None:
            self.destroy_subscription(self.info_sub)
            self.info_sub = None

    def info_callback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        if self.camera_info is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(cv_image)

        if ids is not None:
            K = np.array(self.camera_info.k).reshape((3, 3))
            D = np.array(self.camera_info.d)
            marker_length = 0.05
            object_points = self.get_marker_corners_3d(marker_length)

            for i in range(len(ids)):
                image_points = corners[i][0].astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(object_points, image_points, K, D)
                if success:
                    # cv2.drawFrameAxes(cv_image, K, D, rvec, tvec, 0.03)
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'Depth_Camera_Link'
                    t.child_frame_id = f'aruco_code'
                    t.transform.translation.x = float(tvec[0])
                    t.transform.translation.y = float(tvec[1])
                    t.transform.translation.z = float(tvec[2])

                    q = R.from_rotvec(rvec.flatten()).as_quat()
                    t.transform.rotation.x = float(q[0])
                    t.transform.rotation.y = float(q[1])
                    t.transform.rotation.z = float(q[2])
                    t.transform.rotation.w = float(q[3])

                    self.br.sendTransform(t)

                    msg = Int32()
                    msg.data = int(ids[i][0]) 
                    self.marker_id_pub.publish(msg)

    def get_marker_corners_3d(self, marker_length):
        half_len = marker_length / 2
        return np.array([
            [-half_len,  half_len, 0],
            [ half_len,  half_len, 0],
            [ half_len, -half_len, 0],
            [-half_len, -half_len, 0]
        ], dtype=np.float32)

    def handle_service(self, request, response):
        try:
            if request.enable:
                self.activate_subscribers()
                response.message = "Activate successfully"
            else:
                self.deactivate_subscribers()
                response.message = "Deactivate successfully"
            
            response.success = True
        except Exception as e:
            response.message = f"Failed to call service, Error : {e}"
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()