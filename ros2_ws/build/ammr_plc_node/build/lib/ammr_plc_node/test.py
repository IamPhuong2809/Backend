import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReactBridgeNode(Node):
    def __init__(self):
        super().__init__('react_bridge_node')
        self.subscription = self.create_subscription(
            String,
            'react_topic',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(String, 'react_response', 10)
        self.get_logger().info('ReactBridgeNode started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received from React: {msg.data}')
        response = String()
        response.data = f'Echo: {msg.data.upper()}'
        self.publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = ReactBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
