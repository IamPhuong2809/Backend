import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
import math
import pymcprotocol
from pymcprotocol import Type3E

PLCR = pymcprotocol.Type3E(plctype="iQ-R")
PLCR.setaccessopt(commtype="ascii")

class mobile_robot(Node):
    def __init__(self):
        super().__init__('mobile_robot')
        PLCR.connect("192.168.5.10",5010)  
        #Initialize serial communication
        self.d = 0.35  # Distance between wheels
        self.a = 0.05  # Wheel radius
        
        self.joint_subcriber = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.pose_callback,
            10)

        self.joint_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        self.timer = self.create_timer(0.1, self.read_data)

        # self.goal = self.create_publisher(
        #     PoseStamped,
        #     "/goal_pose",
        #     10
        # )
        # self.timer = self.create_timer(5, self.pose)
    
    def pose(self):
        msg = PoseStamped()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position = Point()
        msg.pose.position.x = -3.5
        msg.pose.position.y = -5.0
        msg.pose.position.z = 0.0

        # Gán orientation
        msg.pose.orientation = Quaternion()
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        print(msg)
        self.goal.publish(msg)

    def pose_callback(self, msg:Twist):
        if not PLCR._is_connected:
            self.get_logger().info("Can't connect to PLC")
            return
        r_val = (1 / self.a) * (msg.linear.x - msg.angular.z * self.d / 2) * (60 / (math.pi * 2))
        l_val = (1 / self.a) * (msg.linear.x + msg.angular.z * self.d / 2) * (60 / (math.pi * 2))
        # message = f"({int(r_val)};{int(l_val)})"
        # print(message)
        PLCR.randomwrite(word_devices=[], word_values=[],
                          dword_devices=["D1032", "D1037"],
                            dword_values=[int(r_val*100000), int(l_val*100000)])
        
    def read_data(self):
        if not PLCR._is_connected:
            self.get_logger().info("Can't connect to PLC")
            return
        msg = JointState()
        pos_addrs = [f"D{addr}" for addr in range(1000, 1044, 5)]
        vel_addrs = [f"D{addr}" for addr in range(1002, 1044, 5)]
        tor_addrs = [f"D{addr}" for addr in range(1005, 1044, 5)]
        data_pos = PLCR.randomread(word_devices=[], dword_devices=pos_addrs)
        vel_pos = PLCR.randomread(word_devices=[], dword_devices=vel_addrs)
        tor_pos = PLCR.randomread(word_devices=[], dword_devices=tor_addrs)
        msg.position = [val / 100000.0 for val in data_pos[1]]
        msg.velocity = [val / 100000.0 for val in vel_pos[1]]
        msg.effort   = [val / 100000.0 for val in tor_pos[1]]
        print(msg)
        # self.joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = mobile_robot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
