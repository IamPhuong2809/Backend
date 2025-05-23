import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class Manipulator(Node):
    def __init__(self):
        super().__init__('Manipulator')
        
        # Subscriber for joint angles
        self.joint_subscription = self.create_subscription(
            Float64MultiArray,
            'joint_angles_move',
            self.joint_callback,
            10)
        
        # Subscriber for robot position
        self.position_subscription = self.create_subscription(
            Float64MultiArray,
            'robot_position_move',
            self.position_callback,
            10)
        
        # Subscriber for tool position
        self.tool_subscription = self.create_subscription(
            Float64MultiArray,
            'tool_position_move',
            self.tool_callback,
            10)
        
        self.tool_subscription = self.create_subscription(
            Float64MultiArray,
            'linear_move',
            self.lin_callback,
            10)
        
        # self.joint_publisher = self.create_publisher(
        #     Float64MultiArray,
        #     'joint_angles_pub',
        #     10)
        
        # self.timer = self.create_timer(0.2, self.read_angle)
        
    def joint_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received joint angles: {msg.data}')

    def position_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received robot positions: {msg.data}')

    def tool_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received tool positions: {msg.data}')

    def lin_callback(self, msg: Float64MultiArray):
        self.get_logger().info(f'Received lin positions: {msg.data}')

    # def read_angle(self):
    #     msg = Float64MultiArray()
    #     msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    #     self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Manipulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
