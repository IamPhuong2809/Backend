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
import threading
from enum import Enum

class PLCConnectionState(Enum):
    DISCONNECTED = 0
    CONNECTING = 1
    CONNECTED = 2
    ERROR = 3

class manipulator(Node):
    def __init__(self):
        super().__init__('plc_node')
        
        # PLC connection parameters
        self.plc_ip = "192.168.5.10"
        self.plc_port = 5010
        self.plc_connection_timeout = 5.0  # seconds
        self.plc_reconnect_interval = 2.0  # seconds
        self.plc_max_retries = 5
        self.plc_retry_count = 0
        
        # PLC connection state
        self.plc_state = PLCConnectionState.DISCONNECTED
        self.plc_lock = threading.Lock()
        
        # Initialize PLC
        self.PLCR = pymcprotocol.Type3E(plctype="iQ-R")
        self.PLCR.setaccessopt(commtype="ascii")
        
        # Start PLC connection
        self.connect_plc()
        
        # PLC monitoring timer
        self.plc_monitor_timer = self.create_timer(1.0, self.monitor_plc_connection)
        
        # Initialize address mappings
        self.pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
        self.vel_addrs_read = [f"D{addr}" for addr in range(1030, 1058, 3)]
        self.tor_addrs_read = [f"D{addr}" for addr in range(1059, 1069, 1)]
        self.pos_addrs_write = [f"D{addr}" for addr in range(5500, 5521, 4)]
        self.vel_addrs_write = [f"D{addr}" for addr in range(5522, 5533, 2)]
        self.jog_addrs_write = [f"D{addr}" for addr in range(5550, 5565, 2)]
        
        # Wheel parameters
        self.d = 0.46  # Distance between wheels
        self.a = 0.161  # Wheel radius
        
        # Subscribers
        self.wheel_subcriber = self.create_subscription(
            Twist,
            '/diff_base_controller/cmd_vel_unstamped',
            self.wheel_callback,
            10)
        
        self.plan_subcriber = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.plan_callback,
            10)
        
        self.joint_move_subscriber = self.create_subscription(
            Float64MultiArray,
            '/joint_angles_move',
            self.joint_move,
            10)
        
        # Action clients
        self.action_grip = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        self.gripper_control = self.create_subscription(
            Float64MultiArray,
            '/grip_control',
            self.action_control_grip,
            10)
        
        # Publishers
        self.ammr_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        # Trajectory control
        self.timerPlan = None
        self.trajectory_points = []
        self.current_index = 0
        
        # Data reading timer
        self.timerData = self.create_timer(0.05, self.read_data)

    def connect_plc(self):
        """Connect to PLC with error handling"""
        with self.plc_lock:
            if self.plc_state == PLCConnectionState.CONNECTING:
                return
            
            self.plc_state = PLCConnectionState.CONNECTING
            self.get_logger().info(f"Attempting to connect to PLC at {self.plc_ip}:{self.plc_port}")
            
            try:
                # Disconnect if already connected
                if hasattr(self.PLCR, '_is_connected') and self.PLCR._is_connected:
                    self.PLCR.close()
                
                # Attempt connection
                self.PLCR.connect(self.plc_ip, self.plc_port)
                
                if self.PLCR._is_connected:
                    self.plc_state = PLCConnectionState.CONNECTED
                    self.plc_retry_count = 0
                    self.get_logger().info("Successfully connected to PLC")
                else:
                    raise Exception("Connection failed - PLC not responding")
                    
            except Exception as e:
                self.plc_state = PLCConnectionState.ERROR
                self.plc_retry_count += 1
                self.get_logger().error(f"Failed to connect to PLC: {str(e)}")
                
                if self.plc_retry_count < self.plc_max_retries:
                    self.get_logger().info(f"Will retry connection in {self.plc_reconnect_interval} seconds (attempt {self.plc_retry_count}/{self.plc_max_retries})")
                else:
                    self.get_logger().error(f"Max retry attempts ({self.plc_max_retries}) reached. Stopping reconnection attempts.")

    def monitor_plc_connection(self):
        """Monitor PLC connection and attempt reconnection if needed"""
        with self.plc_lock:
            if self.plc_state == PLCConnectionState.CONNECTED:
                # Test connection with a simple read
                try:
                    test_read = self.PLCR.randomread(word_devices=["D1000"], dword_devices=[])
                    if test_read[0] != 0:  # Communication error
                        raise Exception(f"PLC communication error: {test_read[0]}")
                except Exception as e:
                    self.get_logger().warning(f"PLC connection lost: {str(e)}")
                    self.plc_state = PLCConnectionState.DISCONNECTED
                    self.plc_retry_count = 0
            
            elif self.plc_state == PLCConnectionState.ERROR:
                if self.plc_retry_count < self.plc_max_retries:
                    time.sleep(self.plc_reconnect_interval)
                    self.connect_plc()
            
            elif self.plc_state == PLCConnectionState.DISCONNECTED:
                self.connect_plc()

    def is_plc_connected(self):
        """Check if PLC is connected and ready"""
        with self.plc_lock:
            return (self.plc_state == PLCConnectionState.CONNECTED and 
                    hasattr(self.PLCR, '_is_connected') and self.PLCR._is_connected)

    def safe_plc_operation(self, operation_func, *args, **kwargs):
        """Safely execute PLC operations with error handling"""
        if not self.is_plc_connected():
            self.get_logger().debug("PLC not connected, skipping operation")
            return None
        
        try:
            with self.plc_lock:
                return operation_func(*args, **kwargs)
        except Exception as e:
            self.get_logger().error(f"PLC operation failed: {str(e)}")
            self.plc_state = PLCConnectionState.DISCONNECTED
            return None

    def wheel_callback(self, msg: Twist):
        """Handle wheel movement commands"""
        def wheel_operation():
            l_val = (msg.linear.x - msg.angular.z * self.d / 2) * (60000) * 1000
            r_val = (msg.linear.x + msg.angular.z * self.d / 2) * (60000) * 1000
            
            self.PLCR.randomwrite(
                word_devices=[], word_values=[],
                dword_devices=["D5546", "D5548"],
                dword_values=[int(r_val), int(l_val)]
            )
            
            message = f"Wheel command: R={int(r_val)}, L={int(l_val)}"
            self.get_logger().debug(message)
            return True
        
        self.safe_plc_operation(wheel_operation)

    def plan_callback(self, msg: DisplayTrajectory):
        """Handle trajectory planning"""
        def plan_operation():
            self.trajectory_points.clear()
            for trajectory in msg.trajectory:
                self.trajectory_points.extend(trajectory.joint_trajectory.points)

            if len(self.trajectory_points) > 200:
                self.get_logger().error("Trajectory too long (>200 points)")
                return False

            if self.timerPlan is not None:
                self.timerPlan.cancel()
            
            # Reset trajectory execution flag
            self.PLCR.randomwrite_bitunits(bit_devices=["M108"], values=[0])

            # Write trajectory points to PLC
            for i in range(200):
                if i >= len(self.trajectory_points):
                    joint_positions = [0, 0, 0, 0, 0, 0]
                    joint_velocities = [0, 0, 0, 0, 0, 0]
                else:  
                    point = self.trajectory_points[i]
                    joint_positions = [int(pos * 180 * 100000 / math.pi) for pos in point.positions]
                    joint_velocities = [abs(int(vel * 1000 * 60 * 180 / math.pi)) for vel in point.velocities]
                
                self.PLCR.randomwrite(
                    word_devices=["D1"], word_values=[i],
                    dword_devices=self.pos_addrs_write + self.vel_addrs_write,
                    dword_values=joint_positions + joint_velocities
                )
                time.sleep(0.01)
            
            # Set trajectory length and start execution
            self.PLCR.randomwrite(
                word_devices=["D2", "D3", "D4", "D5", "D6", "D7", "D8"], 
                word_values=[len(self.trajectory_points) - 1, 0, 0, 0, 0, 0, 0],
                dword_devices=[],
                dword_values=[]
            )
            
            self.PLCR.randomwrite_bitunits(bit_devices=["M106"], values=[1])
            
            self.get_logger().info(f"Trajectory with {len(self.trajectory_points)} points loaded to PLC")
            return True
        
        self.safe_plc_operation(plan_operation)

    def joint_move(self, msg: Float64MultiArray):
        """Handle joint movement commands"""
        def joint_operation():
            value = [int(data * 100000) for data in msg.data]
            self.PLCR.randomwrite(
                word_devices=[], word_values=[],
                dword_devices=self.jog_addrs_write,
                dword_values=value
            )
            return True
        
        self.safe_plc_operation(joint_operation)

    def read_data(self):
        """Read joint data from PLC and publish"""
        def read_operation():
            data_pos = self.PLCR.randomread(word_devices=[], dword_devices=self.pos_addrs_read)
            data_vel = self.PLCR.randomread(word_devices=[], dword_devices=self.vel_addrs_read)
            data_tor = self.PLCR.randomread(word_devices=[], dword_devices=self.tor_addrs_read)
            
            if data_pos[0] != 0 or data_vel[0] != 0 or data_tor[0] != 0:
                raise Exception("PLC read error")
            
            return data_pos[1], data_vel[1], data_tor[1]
        
        result = self.safe_plc_operation(read_operation)
        
        if result is not None:
            data_pos, data_vel, data_tor = result
            
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [
                "Manipulator_J1_Joint", "Manipulator_J2_Joint", "Manipulator_J3_Joint",
                "Manipulator_J4_Joint", "Manipulator_J5_Joint", "Manipulator_J6_Joint",
                "EE_R", "EE_L", "Right_Drive_wheel_Joint", "Left_Drive_wheel_Joint"
            ]
            
            msg.position = [val * math.pi / 18000000.0 for val in data_pos]
            msg.velocity = [val / 60000.0 for val in data_vel]
            msg.effort = [val / 100000.0 for val in data_tor]
            
            self.ammr_publisher.publish(msg)

    def action_control_grip(self, msg: Float64MultiArray):
        """Handle gripper control"""
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

    def destroy_node(self):
        """Clean up resources"""
        try:
            if hasattr(self.PLCR, '_is_connected') and self.PLCR._is_connected:
                self.PLCR.close()
                self.get_logger().info("PLC connection closed")
        except Exception as e:
            self.get_logger().error(f"Error closing PLC connection: {str(e)}")
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = manipulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()