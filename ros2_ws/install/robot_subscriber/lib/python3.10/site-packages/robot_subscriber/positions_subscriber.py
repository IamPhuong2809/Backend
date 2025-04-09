import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class RobotSubscriber(Node):
    def __init__(self):
        super().__init__('positions_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'robot_position', 
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg:Float64MultiArray):
        self.get_logger().info(f'Received robot positions: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()