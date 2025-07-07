import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import Camera
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import time

class ArucoNode(Node):

    ARUCO_DICT = {
	    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
    }

    def __init__(self):
        super().__init__('acuro_node')
        self.dict_to_use = 'DICT_5X5_50'
        self.arucoDict = cv2.aruco.Dictionary_get(ArucoNode.ARUCO_DICT[self.dict_to_use])
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        
        self.marker_length = 0.05  # in meters

        self.br = TransformBroadcaster(self)
        self.camera = Camera()
        self.camera.startStreaming()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frame = self.camera.getNextFrame()
        _, color_image = self.camera.extractImagesFromFrame(frame)
        depth_frame = frame.get_depth_frame()
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        corners, ids, _ = cv2.aruco.detectMarkers(color_image, self.arucoDict, parameters=self.arucoParams)
        if ids is not None:
            fx = depth_intrinsics.fx
            fy = depth_intrinsics.fy
            cx = depth_intrinsics.ppx
            cy = depth_intrinsics.ppy
            
            camera_matrix = np.array([[fx,  0, cx],
                                      [0,  fy, cy],
                                      [0,   0,  1]])
            dist_coeffs = np.zeros((4, 1))

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, camera_matrix, dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]
                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = R.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f'aruco_marker_{marker_id}'

                t.transform.translation.x = float(tvec[0])
                t.transform.translation.y = float(tvec[1])
                t.transform.translation.z = float(tvec[2])
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.br.sendTransform(t)
                self.get_logger().info(f"Published transform for marker {marker_id}")


def main():
    rclpy.init()
    node = ArucoNode()
    rclpy.spin(node)
    node.camera.stopStreaming()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
