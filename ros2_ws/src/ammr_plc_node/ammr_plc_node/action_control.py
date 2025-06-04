# filename: publish_pose_once.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class OneShotPosePublisher(Node):
    def __init__(self):
        super().__init__('one_shot_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/arm_control', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = 1.1
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.9

        msg.pose.orientation.x = 0.502
        msg.pose.orientation.y = 0.502
        msg.pose.orientation.z = 0.498
        msg.pose.orientation.w = 0.498

        self.publisher_.publish(msg)
        self.get_logger().info('Published PoseStamped message once')

        # Hủy timer và shutdown node
        self.timer.cancel()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = OneShotPosePublisher()
    rclpy.spin(node)  # Sẽ tự thoát sau khi publish 1 lần

if __name__ == '__main__':
    main()
