import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointSubscriber(Node):
    def __init__(self):
        super().__init__('joints_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg:Float64MultiArray):
        self.get_logger().info(f'Received joint angles: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()