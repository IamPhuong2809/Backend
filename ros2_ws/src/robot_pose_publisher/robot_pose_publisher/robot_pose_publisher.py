#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose2D
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math

class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__('robot_pose_publisher')
        
        # Tham số cho hai frame
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        
        # Publisher:
        #  - robot_pose: kiểu Pose2D (x,y,yaw)
        #  - odom_pose: kiểu PoseStamped (giữ nguyên)
        self.robot_pose_pub = self.create_publisher(Pose2D, '/robot_pose', 15)
        self.odom_pose_pub  = self.create_publisher(PoseStamped, '/odom_pose', 15)
        
        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.2, self.publish_poses)  # 10 Hz

    def publish_poses(self):
        # 1) robot_pose (Pose2D) từ map -> base_link
        try:
            tf_base = self.tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time())
            
            x = tf_base.transform.translation.x
            y = tf_base.transform.translation.y
            q = tf_base.transform.rotation
            # quaternion -> yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            pose2d = Pose2D()
            pose2d.x = x
            pose2d.y = y
            pose2d.theta = math.degrees(yaw)    # convert radian to degrees
            self.robot_pose_pub.publish(pose2d)
            
            self.get_logger().info(
                f"[RobotPose] x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.3f}°",
                throttle_duration_sec=2.0
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Cannot transform map→{self.base_frame}: {ex}",
                throttle_duration_sec=10.0)

        # 2) odom_pose (PoseStamped) từ map -> odom
        try:
            tf_odom = self.tf_buffer.lookup_transform(
                'map', self.odom_frame, rclpy.time.Time())
            
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = 'map'
            ps.pose.position.x = tf_odom.transform.translation.x
            ps.pose.position.y = tf_odom.transform.translation.y
            ps.pose.position.z = tf_odom.transform.translation.z
            ps.pose.orientation = tf_odom.transform.rotation
            self.odom_pose_pub.publish(ps)
            
            self.get_logger().info(
                f"[OdomPose] x={ps.pose.position.x:.2f}, y={ps.pose.position.y:.2f}",
                throttle_duration_sec=2.0
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Cannot transform map→{self.odom_frame}: {ex}",
                throttle_duration_sec=10.0)

def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

