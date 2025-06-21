import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion, TransformStamped
from std_msgs.msg import Header, Float64MultiArray
import numpy as np
import pymcprotocol
from pymcprotocol import Type3E
import math
import time
# import sys
# import os
# sys.path.append(os.path.abspath('/home/horizon/datn/Backend/ros2_ws/src/ammr_plc_node/ammr_plc_node'))
# from plc_manager_node import get_plc_manager
LimitRangeRobot = [ [0, 180], [0, 180], [0, 135], [0, 180], [0, 180], [0, 359.99],]

class manipulator(Node):
    def __init__(self):
        super().__init__('plc_node')
        self.PLCR = pymcprotocol.Type3E(plctype = "iQ-R")
        self.PLCR.setaccessopt(commtype="binary")
        self.connect = False

        # self.plc_manager = get_plc_manager()

        #Initialize serial communication
        self.vel_mobile_read = ["D1054", "D1057"]
        self.pos_addrs_read = [f"D{addr}" for addr in range(1000, 1022, 3)] + ["D32336", "D32384"]
        self.vel_addrs_read = [f"D{addr}" for addr in range(1030, 1058, 3)]
        self.transmission_ratio = [150, 100, 112, 7, 100, 9, 1, 1, 43.75, 43.75]
        self.tor_addrs_read = [f"D{addr}" for addr in range(1059, 1069, 1)]
        self.pos_addrs_write = [f"D{addr}" for addr in range(5500, 5521, 4)]
        self.vel_addrs_write = [f"D{addr}" for addr in range(5522, 5533, 2)]
        self.jogpos_addrs_write = [f"D{addr}" for addr in range(5550, 5561, 2)]
        self.jogvel_addrs_write = ["D5562", "D5564"]
        #region Wheel sub
        self.d = 0.48 # Distance between wheels
        self.a = 0.161 # Wheel radius
        
        self.wheel_subcriber = self.create_subscription(
            Twist,
            '/cmd_vel_plc',
            # '/diff_base_controller/cmd_vel_unstamped',
            self.wheel_callback,
            10)

        self.wheel_subcriber = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.wheel_callback,
            10)

        self.pub_vel_unstampd = self.create_publisher(
            Twist,
            '/diff_base_controller/cmd_vel_unstamped',
            10)
        
        #endregion

        #region Joint sub
        self.plan_subcriber = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.plan_callback,
            10)
        
        self.trajectory_points = []

        self.plan_subcriber = self.create_subscription(
            Float64MultiArray,
            '/joint_angles_move',
            self.joint_move,
            10)
        #endregion

        #region Action and Sub control gripper
        self.action_grip = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        self.gripper_control = self.create_subscription(
            Float64MultiArray,
            '/grip_control',
            self.action_control_grip,
            10)
        #endregion
        
        self.ammr_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        self.timerData = self.create_timer(0.2, self.read_data)

        # self.goal = self.create_publisher(
        #     PoseStamped,
        #     "/goal_pose",
        #     10
        # )
        # self.timer = self.create_timer(5, self.pose)

    def connect_PLC(self):
        self.PLCR.connect("192.168.5.10", 5010)
        if self.PLCR._is_connected:
            print("[PLC Manager] Connected to PLC at 192.168.5.10:5010")
            self.PLCR._sock.settimeout(10.0)
            self.connect = True
        else:
            print("[PLC Manager] Can't connect to PLC")
            self.connect = False
    
    def navigation_callback(self, msg:Twist):
        self.pub_vel_unstampd.publish(msg)

    def wheel_callback(self, msg:Twist):
        if not self.PLCR._is_connected:
            self.connect_PLC()
            return

        l_val = (msg.linear.x - msg.angular.z * self.d / 2) * (60000) * 100 #(1 / self.a) * 
        r_val = (msg.linear.x + msg.angular.z * self.d / 2) * (60000) * 100
        # message = f"({int(r_val)};{int(l_val)})"
        # print(message)
        try:
            self.PLCR.randomwrite(word_devices=[], word_values =[], dword_devices=["D5546", "D5548"], dword_values=[int(r_val), int(l_val)])
        except:
            self.PLCR.remote_reset()
            print("Failed to write whell")
            return
        
    def plan_callback(self, msg : DisplayTrajectory):
        if not self.PLCR._is_connected:
            self.connect_PLC()
            return

        self.trajectory_points.clear()
        for trajectory in msg.trajectory:
            self.trajectory_points.extend(trajectory.joint_trajectory.points)
        
        if self.trajectory_points:
            point = self.trajectory_points[-1]  # Lấy điểm cuối cùng trong danh sách

            joint= [int(pos * 180 * 100000 / math.pi) for pos in point.positions]
            joint = joint

            print(f"End Point -> pos: {joint}")

            try:
                self.PLC.randomwrite(
                    word_devices=[], word_values=[],  # Có thể bỏ hoặc đặt index tùy ý
                    dword_devices=self.pos_addrs_write,
                    dword_values=joint
                )
            except:
                self.PLCR.remote_reset()
                print("Failed to write Move Pos")
                return
        

    def joint_move(self, msg: Float64MultiArray):
        if not self.PLCR._is_connected:
            self.connect_PLC()
            return

        data = msg.data
        for i, val in enumerate(data):
            min_val, max_val = LimitRangeRobot[i]
            if not (min_val <= val <= max_val):
                print(f"[!] Joint {i+1} value {val} out of range [{min_val}, {max_val}]")
                return  # Không write nếu có giá trị sai

        value = [int(d * 100000) for d in data]
        print(value)
        try:
            self.PLCR.randomwrite(
                word_devices=[], word_values=[],
                dword_devices=self.jogpos_addrs_write,
                dword_values=value
            )
        except:
            self.PLCR.remote_reset()
            print("Failed to write Jog Joint")
            return
    
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
        self.goal.publish(msg)
        
    def read_data(self):
        if not self.PLCR._is_connected:
            self.connect_PLC()
            return

        msg = JointState()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "Manipulator_J1_Joint",
            "Manipulator_J2_Joint",
            "Manipulator_J3_Joint",
            "Manipulator_J4_Joint",
            "Manipulator_J5_Joint",
            "Manipulator_J6_Joint",
            "EE_R",
            "EE_L",
            "Right_Drive_wheel_Joint",
            "Left_Drive_wheel_Joint"
        ]

        msg_wheel = Twist()
        try:
            data_pos = self.PLCR.randomread( word_devices=[], dword_devices=self.pos_addrs_read)
            data_vel = self.PLCR.randomread( word_devices=[], dword_devices=self.vel_mobile_read)
            # data_tor = self.PLCR.randomread( word_devices=[], dword_devices=self.tor_addrs_read)
        except:
            self.PLCR.remote_reset()
            print("Failed to read Joint States")
            return
        pos1_8 = [val * math.pi / 18000000.0 for val in data_pos[1][:8]]
        pos9_10 = [val / (10000000.0 * self.a) for val in data_pos[1][-2:]]
        msg.position = pos1_8 + pos9_10
        
        vel = [float(val* (math.pi * self.a )/ (3000.0 * 43.75))  for val in data_vel[1]] #m/s
        msg_wheel.linear.x = (vel[0] + vel[1])/2
        msg_wheel.angular.z = (vel[0] - vel[1])/self.d
        self.pub_vel_unstampd.publish(msg_wheel)
        self.ammr_publisher.publish(msg)
        # msg.velocity = [(val * math.pi * self.a ) / (3000.0 * self.transmission_ratio[i]) for i, val in enumerate(data_vel[1])]
        # msg.effort   = [val / 100000.0 for val in data_tor[1]]

    def action_control_grip(self, msg : Float64MultiArray):
        data = msg.data
        if len(data) != 2:
            self.get_logger().error("Grip data must contain 2 elements: [position, max_effort]")
            return

        position = data[0]
        max_effort = data[1]

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f"Sending gripper command: position={position}, max_effort={max_effort}")
        self.action_grip.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = manipulator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
