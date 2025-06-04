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
import threading

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
        self.subscription_status = self.node.create_subscription(
            String, '/arm_status', self.listener_callback, 10
        )
        
        self.success = False
    
    def listener_callback(self, msg: String):
        self.success = "error" not in msg.data
        print(f"[ROS CALLBACK] arm_status received: {msg.data} -> success = {self.success}")
    
    def create_pose_message(self, x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg):
        x = x_mm / 100.0 
        y = y_mm / 100.0
        z = z_mm / 100.0
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
    
    def validate_joint_data(self, data_array, expected_length=8):
        if len(data_array) != expected_length:
            raise ValueError(f"Expected {expected_length} values, got {len(data_array)}")
        return [float(j) for j in data_array]

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

def start_ros_spin():
    controller.spin_ros_node()

@api_view(['POST'])
def O0025(request):
    data = request.data 
    dataArray = data.get("joint") 
    dataArray += [data.get("velocity"), data.get("acceleration")]
    jogMode = data.get("jogMode")

    validated_data = controller.validate_joint_data(dataArray)
    
    if jogMode == "Work":
        if all(item == 0 for item in dataArray[:6]):
            return Response(status=status.HTTP_204_NO_CONTENT)
        msg = controller.create_pose_message(*validated_data[:6])
        controller.publisher_work.publish(msg)
    elif jogMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        controller.publisher_joint.publish(msg)
    elif jogMode == "Tool":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        controller.publisher_tool.publish(msg)
    
    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0021(request):
    print("abort")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0022(request):
    data = request.data 
    dataArray = data.get("joint") 
    dataArray += [data.get("velocity"), data.get("acceleration")]
    moveMode = data.get("moveMode") 

    validated_data = controller.validate_joint_data(dataArray)

    if moveMode == "LIN":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        controller.publisher_lin.publish(msg)
    elif moveMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        controller.publisher_joint.publish(msg)
    elif moveMode == "PTP":
        if all(item == 0 for item in dataArray[:6]):
            return Response(status=status.HTTP_204_NO_CONTENT)
        msg = controller.create_pose_message(*validated_data[:6])
        print(msg)
        controller.publisher_work.publish(msg)
    
    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0023(request):
    print("abort")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0024(request):
    print("home")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0027(request):
    data = request.data
    try:
        id = data.get("id")
        name = data.get("name")
        joint = [float(j) for j in data.get("joint")] 
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
        joint = [float(j) for j in data.get("joint")] 
        max_point_id = Point.objects.filter(path_id=id_path).aggregate(Max("point_id"))["point_id__max"] or 0
        if id == max_point_id + 1:
            updated = Point.objects.create( point_id=id, name = name, x = joint[0], y = joint[1], z = joint[2],
                roll = joint[3], pitch = joint[4], yaw = joint[5], tool=0, figure=0, work=0, motion="LIN",
                cont=False, stop=False, vel=0, acc=0, corner=0, path_id=id_path
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

