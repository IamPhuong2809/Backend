# tf_map_to_base.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String

class MapToBasePublisher(Node):
    def __init__(self):
        super().__init__('map_to_base_link_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TransformStamped, '/map_to_base_link', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.publisher.publish(tf)
            self.get_logger().info(f"Published TF map→base_link: {tf.transform.translation}")
        except Exception as e:
            self.get_logger().warn(f"Cannot lookup transform: {e}")

def main():
    rclpy.init()
    node = MapToBasePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
