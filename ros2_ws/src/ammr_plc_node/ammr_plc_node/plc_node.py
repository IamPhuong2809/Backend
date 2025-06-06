import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from std_msgs.msg import Header, Float64MultiArray
import pymcprotocol
from pymcprotocol import Type3E
import math
import time

PLCR = pymcprotocol.Type3E(plctype="iQ-R")
PLCR.setaccessopt(commtype="ascii")

class manipulator(Node):
    def __init__(self):
        super().__init__('plc_node')
        PLCR.connect("192.168.5.10",5010)  
        #Initialize serial communication
        self.pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
        self.vel_addrs_read = [f"D{addr}" for addr in range(1030, 1058, 3)]
        self.tor_addrs_read = [f"D{addr}" for addr in range(1059, 1069, 1)]
        self.pos_addrs_write = [f"D{addr}" for addr in range(5500, 5521, 4)]
        self.vel_addrs_write = [f"D{addr}" for addr in range(5522, 5533, 2)]
        self.jog_addrs_write = [f"D{addr}" for addr in range(5534, 5545, 2)]
        #region Wheel sub
        self.d = 0.46 # Distance between wheels
        self.a = 0.161 # Wheel radius
        
        self.wheel_subcriber = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
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
        
        self.timerData = self.create_timer(0.05, self.read_data)

        # self.goal = self.create_publisher(
        #     PoseStamped,
        #     "/goal_pose",
        #     10
        # )
        # self.timer = self.create_timer(5, self.pose)

    def wheel_callback(self, msg:Twist):
        if not PLCR._is_connected:
            self.get_logger().info("Can't connect to PLC")
            return
        l_val = (msg.linear.x - msg.angular.z * self.d / 2) * (60000) #(1 / self.a) * 
        r_val = (msg.linear.x + msg.angular.z * self.d / 2) * (60000)
        # message = f"({int(r_val)};{int(l_val)})"
        # print(message)
        PLCR.randomwrite(word_devices=[], word_values=[],
                          dword_devices=["D5546", "D5548"],
                            dword_values=[int(r_val), int(l_val)])
        
    def plan_callback(self, msg : DisplayTrajectory):
        if not PLCR._is_connected:
            self.get_logger().info("Can't connect to PLC")
            return
        
        self.trajectory_points.clear()
        for trajectory in msg.trajectory:
            self.trajectory_points.extend(trajectory.joint_trajectory.points)
        self.current_index = 2
        print(len(self.trajectory_points))

        if self.timerPlan is not None:
            self.timerPlan.cancel()

        for i in range(200):
            if i >= len(self.trajectory_points):
                joint_positions = [0, 0, 0, 0, 0, 0]
                joint_velocities = [0, 0, 0, 0, 0, 0]

            else:  
                point = self.trajectory_points[i]
                joint_positions = [int(pos * 180 *100000 / math.pi)  for pos in point.positions]
                joint_velocities = [abs(int(vel * 1000 * 60 * 180 / math.pi)) for vel in point.velocities]
            
            print(f"{i}, pos {joint_positions}, vel {joint_velocities}")

            PLCR.randomwrite(
                word_devices=["D1"], word_values=[i],
                dword_devices=self.pos_addrs_write + self.vel_addrs_write,
                dword_values=joint_positions + joint_velocities
            )
            time.sleep(0.01)
        
        PLCR.randomwrite(
            word_devices=["D2", "D3", "D4", "D5", "D6", "D7", "D8"], word_values=[len(self.trajectory_points) - 1, 0, 0, 0, 0, 0, 0],
            dword_devices=[],
            dword_values=[]
        )

        PLCR.randomwrite_bitunits(bit_devices=["M108"], values=[0])
    
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
        if not PLCR._is_connected:
            self.get_logger().info("Can't connect to PLC")
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

        data_pos = PLCR.randomread(word_devices=[], dword_devices=self.pos_addrs_read)
        data_vel = PLCR.randomread(word_devices=[], dword_devices=self.vel_addrs_read)
        data_tor = PLCR.randomread(word_devices=[], dword_devices=self.tor_addrs_read)
        msg.position = [val * math.pi / 18000000.0 for val in data_pos[1]]
        msg.velocity = [val / 60000.0 for val in data_vel[1]]
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
