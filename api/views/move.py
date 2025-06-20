from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat
from api.models import Point,Global
from django.db.models import Max
import math
import time
import threading
from api.views.plc_manager import get_plc_manager
from api.views.components import robotData
from api.views.Kinematics import quaternion_ik

plc_manager = get_plc_manager()

class ManipulatorController:
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('Manipulator')
        
        # Publishers
        self.publisher_joint = self.node.create_publisher(Float64MultiArray, 'joint_angles_move', 10)
        self.publisher_work = self.node.create_publisher(PoseStamped, 'arm_control', 10)
        self.publisher_tool = self.node.create_publisher(Float64MultiArray, 'tool_position_move', 10)
        self.publisher_lin = self.node.create_publisher(Float64MultiArray, 'linear_move', 10)
        
        # Subscriber
        # self.subscription_status = self.node.create_subscription(
        #     String, '/arm_status', self.listener_callback, 10
        # )
        
        self.success = False
    
    def listener_callback(self, msg: String):
        self.success = "error" not in msg.data
        print(f"[ROS CALLBACK] arm_status received: {msg.data} -> success = {self.success}")
    
    def create_pose_message(self, x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg):
        x = x_mm / 1000.0 
        y = y_mm / 1000.0
        z = z_mm / 1000.0
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)

        qw, qx, qy, qz = euler2quat(roll, pitch, yaw)

        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        return msg

    def spin_ros_node(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.node.destroy_node()
            rclpy.shutdown()

# Khởi tạo controller và chạy thread ROS spin
controller = ManipulatorController()
LimitRangeRobot = [ [0, 180], [0, 180], [0, 135], [0, 180], [0, 180], [0, 359],]

def start_ros_spin():
    controller.spin_ros_node()

@api_view(['POST'])
def O0025(request):
    data = request.data 
    dataJoint = data.get("joint") 
    dataArray = [data.get("velocity"), data.get("acceleration")]
    jogMode = data.get("jogMode")
    all_zero = all(int(value) == 0 for value in dataJoint)
    if all_zero:
        return Response({"success": True}, status=status.HTTP_200_OK)

    joint = [0, 0, 0, 0, 0, 0]
    keys = ['t1', 't2', 't3', 't4', 't5', 't6']
    for i, key in enumerate(keys):
        joint[i] = robotData["jointCurrent"][key]

    if jogMode == "Work":
        if all(item == 0 for item in dataJoint):
            return Response(status=status.HTTP_204_NO_CONTENT)
        msg = Float64MultiArray()
        success, theta, _ , _ , _ , _ = quaternion_ik(dataJoint, joint)
        if success:
            msg.data = [math.degrees(float(j)) for j in theta]
            controller.publisher_joint.publish(msg)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)
    elif jogMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [round(float(j),3) for j in dataJoint]
        controller.publisher_joint.publish(msg)
    elif jogMode == "Tool":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        controller.publisher_tool.publish(msg)
    
    return Response({"success": True}, status=status.HTTP_200_OK)

@api_view(['GET'])
def O0021(request):
    print("abort")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0022(request):
    data = request.data 
    dataJoint = data.get("joint") 
    dataArray = [data.get("velocity"), data.get("acceleration")]
    moveMode = data.get("moveMode") 

    joint = [0, 0, 0, 0, 0, 0]
    keys = ['t1', 't2', 't3', 't4', 't5', 't6']
    for i, key in enumerate(keys):
        joint[i] = robotData["jointCurrent"][key]

    if moveMode == "LIN":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        controller.publisher_lin.publish(msg)
    elif moveMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataJoint] 
        controller.publisher_joint.publish(msg)
    elif moveMode == "PTP":
        if all(item == 0 for item in dataJoint):
            return Response(status=status.HTTP_204_NO_CONTENT)
        msg = Float64MultiArray()
        success, theta, _ , _ , _ , _ = quaternion_ik(dataJoint, joint)
        if success:
            msg.data = [math.degrees(float(j)) for j in theta]
            controller.publisher_joint.publish(msg)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)
    
    return Response({"success": True}, status=status.HTTP_200_OK)

@api_view(['GET'])
def O0023(request):
    plc_manager.rising_pulse(device_name=['M116'])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0024(request):
    print("home")
    plc_manager.write_device_block(device_name=['M108'], values=[1])
    time.sleep(0.05)
    plc_manager.write_device_block(device_name=["M102"], values=[1])
    plc_manager.write_device_block(device_name=['M108'],values=[0])
    jog_addrs_write = [f"D{addr}" for addr in range(5550, 5563, 2)]
    joint = [0, 0, 0, 0, 0, 0, 200000]
    keys = ['t1', 't2', 't3', 't4', 't5', 't6']
    for i, key in enumerate(keys):
        joint[i] = int(robotData["jointCurrent"][key]*100000)
    plc_manager.write_random(
        dword_devices=jog_addrs_write,
        dword_values=joint
    )
    plc_manager.write_device_block(device_name=['M113'],values=[0])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def jog_mode(request):
    type = request.data.get("type")
    plc_manager.rising_pulse(device_name=["M108"])
    jog_addrs_write = [f"D{addr}" for addr in range(5550, 5563, 2)]
    joint = [0, 0, 0, 0, 0, 0, 200000]
    keys = ['t1', 't2', 't3', 't4', 't5', 't6']
    for i, key in enumerate(keys):
        joint[i] = int(robotData["jointCurrent"][key]*100000)
    plc_manager.write_random(
        dword_devices=jog_addrs_write,
        dword_values=joint
    )
    plc_manager.rising_pulse(device_name=["M113"])

    # elif type == "Work" or type == "PTP":
    #     plc_manager.write_device_block(device_name=["M112"], values=[1])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0027(request):
    data = request.data
    try:
        id = data.get("id")
        name = data.get("name")
        joint = [round(float(j),3) for j in data.get("joint")] 
        max_point_id = Global.objects.aggregate(Max("point_id"))["point_id__max"] or 0
        if id == max_point_id + 1:
            updated = Global.objects.create(point_id=id, name=name, x=joint[0], y=joint[1], z=joint[2], 
                                            roll=joint[3], pitch=joint[4], yaw=joint[5], tool=0, figure=0, work=0)
        else:
            updated = Global.objects.filter(point_id=id).update(
                name = name,
                x = joint[0],
                y = joint[1],
                z = joint[2],
                roll = joint[3],
                pitch = joint[4],
                yaw = joint[5]
            )

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False, "error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

    except Exception as e:
        return Response({"success": False, "error": str(e)}, status=status.HTTP_400_BAD_REQUEST)
    
@api_view(['POST'])
def O0028(request):
    data = request.data
    try:
        id_path = data.get("id_parent")
        id = data.get("id")
        name = data.get("name")
        joint = [round(float(j),3) for j in data.get("joint")] 
        max_point_id = Point.objects.filter(path_id=id_path).aggregate(Max("point_id"))["point_id__max"] or 0
        if id == max_point_id + 1:
            updated = Point.objects.create( point_id=id, name = name, x = joint[0], y = joint[1], z = joint[2],
                roll = joint[3], pitch = joint[4], yaw = joint[5], tool=0, figure=0, work=0, motion="LIN",
                ee="SKIP", stop=False, vel=0, acc=0, corner=0, path_id=id_path
            )
        else:
            updated = Point.objects.filter(path_id=id_path,point_id=id).update(
                name = name,
                x = joint[0],
                y = joint[1],
                z = joint[2],
                roll = joint[3],
                pitch = joint[4],
                yaw = joint[5]
            )

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False, "error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

    except Exception as e:
        return Response({"success": False, "error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

