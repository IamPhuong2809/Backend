import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CombinedSubscriber(Node):
    def __init__(self):
        super().__init__('API_move')
        
        # Subscriber for joint angles
        self.joint_subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles',
            self.joint_callback,
            10)
        
        # Subscriber for robot position
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            'robot_position',
            self.position_callback,
            10)
        
        # Subscriber for tool position
        self.tool_subscription = self.create_subscription(
            Float64MultiArray,
            'tool_position',
            self.tool_callback,
            10)
        
        self.tool_subscription = self.create_subscription(
            Float64MultiArray,
            'linear_move',
            self.lin_callback,
            10)
        
    def joint_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received joint angles: {msg.data}')

    def position_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received robot positions: {msg.data}')

    def tool_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received tool positions: {msg.data}')

    def lin_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received lin positions: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CombinedSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
