from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from ammr_moveit_controller.srv import MoveRobot
from geometry_msgs.msg import PoseStamped
from transforms3d.euler import euler2quat
from api.models import Point,Global
from django.db.models import Max
import math
import threading
import time
import numpy as np
from api.views.plc_manager import get_plc_manager
from api.views.components import robotData
from api.views.Kinematics import quaternion_ik
from transforms3d.euler import euler2quat

plc_manager = get_plc_manager()

class ManipulatorController(Node):
    def __init__(self):
        super().__init__('django_ros_client')
        self.cli = self.create_client(MoveRobot, '/ammr_moveit_controller/move_robot')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available...')
        self.req = MoveRobot.Request()

    def call_move_robot(self, mode, target):
        self.req.mode = mode
        self.req.target = target
        future = self.cli.call_async(self.req)
        print(self.req)
        while not future.done():
            time.sleep(0.01)

        if future.result():
            return future.result().success, future.result().message
        else:
            return False, "Service call failed"

    def spin_ros_node(self):
        rclpy.spin(self)

# Khởi tạo controller và chạy thread ROS spin
if not rclpy.ok():
    rclpy.init()
controller = ManipulatorController()
threading.Thread(target=controller.spin_ros_node, daemon=True).start()

LimitRangeRobot = [ [0, 180], [0, 180], [0, 135], [0, 180], [0, 180], [0, 359],]

def rpy_to_quaternion(rpy):
    roll, pitch, yaw = [math.radians(a) for a in rpy]  
    qw, qx, qy, qz = euler2quat(roll, pitch, yaw)  
    return qx, qy, qz, qw

@api_view(['POST'])
def O0025(request):
    data = request.data 
    dataJoint = data.get("joint") 
    dataArray = [data.get("velocity"), data.get("acceleration")]
    jogMode = data.get("jogMode")
    all_zero = all(int(value) == 0 for value in dataJoint)
    if all_zero:
        return Response({"success": True}, status=status.HTTP_200_OK)

    if jogMode == "Work":
        if all(item == 0 for item in dataJoint):
            return Response(status=status.HTTP_204_NO_CONTENT)
        xyz = xyz = [pos/1000 for pos in dataJoint[:3]]
        rpy = dataJoint[3:]
        qx, qy, qz, qw = rpy_to_quaternion(rpy)
        target = xyz + [qx, qy, qz, qw] 
        success, message = controller.call_move_robot("p2p", target)
        if not success:
            return Response({"success": False}, status=status.HTTP_200_OK)
    elif jogMode =="Joint":
        theta = [round(float(j),3) for j in dataJoint]
        plc_manager.move_joint_degree(theta)
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

    all_zero = all(int(value) == 0 for value in dataJoint)
    if all_zero:
        return Response({"success": True}, status=status.HTTP_200_OK)

    joint = [0, 0, 0, 0, 0, 0]
    keys = ['x', 'y', 'z', 'rl', 'pt', 'yw']
    for i, key in enumerate(keys):
        joint[i] = robotData["positionCurrent"][key]
    if moveMode == "LIN":
        diff = [target - current for target, current in zip(dataJoint, joint)]
        position_diff = diff[:3]
        rotation_diff = diff[3:]
        if any(abs(val) > 1 for val in rotation_diff) and any(abs(val) > 10 for val in position_diff):
            xyz = [round(pos/1000, 2) for pos in dataJoint[:3]]
            rpy = dataJoint[3:]
            qx, qy, qz, qw = rpy_to_quaternion(rpy)
            target_rot = xyz + [qx, qy, qz, qw] 
            success, message = controller.call_move_robot("p2p", target_rot) 

            timeout = 10 
            start_time = time.time()    
            while robotData.get("busy", True):
                if time.time() - start_time > timeout:
                    break
                time.sleep(0.1)
            target_pos = [round(val / 1000, 2) for val in position_diff]
            success, message = controller.call_move_robot("cartesian", target_pos)
        elif any(abs(val) > 1 for val in rotation_diff):
            xyz = [round(pos/1000, 2) for pos in dataJoint[:3]]
            rpy = dataJoint[3:]
            qx, qy, qz, qw = rpy_to_quaternion(rpy)
            target = xyz + [qx, qy, qz, qw] 
            success, message = controller.call_move_robot("p2p", target) 
        elif any(abs(val) > 10 for val in position_diff):
            target = [round(val / 1000, 2) for val in position_diff]
            success, message = controller.call_move_robot("cartesian", target)
        if not success:
            return Response({"success": False}, status=status.HTTP_200_OK)
    elif moveMode =="Joint":
        theta = [round(float(j),3) for j in dataJoint]
        plc_manager.move_joint_degree(theta)
    elif moveMode == "PTP":
        xyz = [pos/1000 for pos in dataJoint[:3]]
        rpy = dataJoint[3:]
        qx, qy, qz, qw = rpy_to_quaternion(rpy)
        target = xyz + [qx, qy, qz, qw] 
        success, message = controller.call_move_robot("p2p", target)
        if not success:
            return Response({"success": False}, status=status.HTTP_200_OK)
    
    return Response({"success": True}, status=status.HTTP_200_OK)

@api_view(['GET'])
def O0023(request):
    plc_manager.rising_pulse(device_name=['M116'])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0024(request):
    data = request.data
    jogMode = data.get("jogMode")
    moveMode = data.get("moveMode")
    if jogMode == "Joint" or moveMode == "Joint":
        jog_addrs_write = [f"D{addr}" for addr in range(2500, 2513, 2)]
        joint = [9000000, 9000000, 4500000, 9000000, 9000000, 18000000, 200000]
        plc_manager.write_random(
            dword_devices=jog_addrs_write,
            dword_values=joint
        )
    else:
        target = [np.pi/2, np.pi/2, np.pi/4, np.pi/2, np.pi/2, np.pi]
        controller.call_move_robot("joint", target)

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def jog_mode(request):
    type_data = request.data.get("type")
    if type_data == "Joint":
        jog_addrs_write = [f"D{addr}" for addr in range(2500, 2513, 2)]
        joint = [0, 0, 0, 0, 0, 0, 200000]
        keys = ['t1', 't2', 't3', 't4', 't5', 't6']
        for i, key in enumerate(keys):
            joint[i] = int(robotData["jointCurrent"][key]*100000)
        plc_manager.write_random(
            dword_devices=jog_addrs_write,
            dword_values=joint
        )
        plc_manager.write_device_block(device_name=["M206"], values=[1])
    else:
        plc_manager.write_device_block(device_name=["M204"], values=[1])

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
            updated = Global.objects.create(point_id=id, name=name, t1 = joint[0], t2 = joint[1], t3 = joint[2],
                                    t4 = joint[3], t5 = joint[4], t6 = joint[5], tool=0, figure=0, work=0)
        else:
            updated = Global.objects.filter(point_id=id).update(
                name = name,
                t1= joint[0],
                t2 = joint[1],
                t3 = joint[2],
                t4 = joint[3],
                t5 = joint[4],
                t6 = joint[5]
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
            updated = Point.objects.create( point_id=id, name = name, t1 = joint[0], t2 = joint[1], t3 = joint[2],
                t4 = joint[3], t5 = joint[4], t6 = joint[5], tool=0, figure=0, work=0, motion="LIN",
                ee="SKIP", stop=False, vel=0, acc=0, corner=0, path_id=id_path
            )
        else:
            updated = Point.objects.filter(path_id=id_path,point_id=id).update(
                name = name,
                t1= joint[0],
                t2 = joint[1],
                t3 = joint[2],
                t4 = joint[3],
                t5 = joint[4],
                t6 = joint[5]
            )

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False, "error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

    except Exception as e:
        return Response({"success": False, "error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

