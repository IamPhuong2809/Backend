import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import Header, Float64MultiArray
import numpy as np
import pymcprotocol
from pymcprotocol import Type3E
import math
import time
import sys
import os
sys.path.append(os.path.abspath('/home/horizon/datn/Backend/ros2_ws/src/ammr_plc_node/ammr_plc_node'))
from plc_manager_node import get_plc_manager

class manipulator(Node):
    def __init__(self):
        super().__init__('plc_node')

        self.plc_manager = get_plc_manager()

        #Initialize serial communication
        self.pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
        self.vel_addrs_read = [f"D{addr}" for addr in range(1030, 1058, 3)]
        self.transmission_ratio = [150, 100, 112, 7, 100, 9, 1, 1, 43.75, 43.75]
        self.tor_addrs_read = [f"D{addr}" for addr in range(1059, 1069, 1)]
        self.pos_addrs_write = [f"D{addr}" for addr in range(5500, 5521, 4)]
        self.vel_addrs_write = [f"D{addr}" for addr in range(5522, 5533, 2)]
        self.jog_addrs_write = [f"D{addr}" for addr in range(5550, 5565, 2)]
        #region Wheel sub
        self.d = 0.46 # Distance between wheels
        self.a = 0.161 # Wheel radius
        
        self.wheel_subcriber = self.create_subscription(
            Twist,
            '/diff_base_controller/cmd_vel_unstamped',
            self.wheel_callback,
            10)

        self.wheel_subcriber = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.wheel_callback,
            10)
        #endregion

        #region Joint sub
        self.plan_subcriber = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.plan_callback,
            10)
        
        self.timerPlan = None
        self.trajectory_points = []
        self.current_index = 0

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

    def wheel_callback(self, msg:Twist):
        l_val = (msg.linear.x - msg.angular.z * self.d / 2) * (60000) * 1000 #(1 / self.a) * 
        r_val = (msg.linear.x + msg.angular.z * self.d / 2) * (60000) * 1000
        message = f"({int(r_val)};{int(l_val)})"
        # print(message)
        self.plc_manager.write_random(dword_devices=["D5546", "D5548"], dword_values=[int(r_val), int(l_val)])
        print(message)
        
    def plan_callback(self, msg : DisplayTrajectory):
        self.trajectory_points.clear()
        for trajectory in msg.trajectory:
            self.trajectory_points.extend(trajectory.joint_trajectory.points)

        if len(self.trajectory_points) > 200:
            print("Out of range")
            return

        if self.timerPlan is not None:
            self.timerPlan.cancel()
        
        self.plc_manager.write_device_block(device_name=["M108"], values=[0])

        for i in range(200):
            if i >= len(self.trajectory_points):
                joint_positions = [0, 0, 0, 0, 0, 0]
                joint_velocities = [0, 0, 0, 0, 0, 0]

            else:  
                point = self.trajectory_points[i]
                joint_positions = [int(pos * 180 *100000 / math.pi)  for pos in point.positions]
                joint_velocities = [abs(int(vel * 1000 * 60 * 180 / math.pi)) for vel in point.velocities]
            
            print(f"{i}, pos {joint_positions}, vel {joint_velocities}")

            self.plc_manager.write_random(
                word_devices=["D1"], word_values=[i],
                dword_devices=self.pos_addrs_write + self.vel_addrs_write,
                dword_values=joint_positions + joint_velocities
            )
            time.sleep(0.01)
        
        self.plc_manager.write_random(word_devices=["D2", "D3", "D4", "D5", "D6", "D7", "D8"], word_values=[len(self.trajectory_points) - 1, 0, 0, 0, 0, 0, 0],)

        self.plc_manager.write_random(device_name=["M106"], values=[1])

    def joint_move(self, msg: Float64MultiArray):
        value = [int(data*100000) for data in msg.data]
        self.plc_manager.write_random(
            word_devices=[], word_values=[],
            dword_devices=self.jog_addrs_write,
            dword_values=value
        )
    
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
        data_pos = self.plc_manager.read_random(dword_devices=self.pos_addrs_read)
        data_vel = self.plc_manager.read_random(dword_devices=self.vel_addrs_read)
        data_tor = self.plc_manager.read_random(dword_devices=self.tor_addrs_read)
        if data_pos[1] != None and data_vel != None and data_tor != None:
            msg.position = [val * math.pi / 18000000.0 for val in data_pos[1]]
            msg.velocity = [np.deg2rad(val * self.transmission_ratio[i]  / 100.0) for i, val in enumerate(data_vel[1])]
            msg.effort   = [val / 100000.0 for val in data_tor[1]]
            self.ammr_publisher.publish(msg)

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
