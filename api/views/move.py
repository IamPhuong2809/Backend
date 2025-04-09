from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

rclpy.init()

node = rclpy.create_node('API_move')
publisher_joint = node.create_publisher(Float64MultiArray, 'joint_angles', 10)
publisher_work = node.create_publisher(Float64MultiArray, 'robot_position', 10)
publisher_tool = node.create_publisher(Float64MultiArray, 'tool_position', 10)
publisher_lin = node.create_publisher(Float64MultiArray, 'linear_move', 10)

@api_view(['POST'])
def O0025(request):
    data = request.data 
    dataArray = data.get("joint") 
    dataArray += [data.get("velocity"), data.get("acceleration")]
    jogMode = data.get("jogMode")

    if len(dataArray) != 8:
        return Response({"error": "Missing 'joint' value"}, status=status.HTTP_400_BAD_REQUEST)

    if jogMode == "Work":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        publisher_work.publish(msg)
    elif jogMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        publisher_joint.publish(msg)
    elif jogMode == "Tool":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray]
        publisher_tool.publish(msg)
    
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

    if len(dataArray) != 8:
        return Response({"error": "Missing 'joint' value"}, status=status.HTTP_400_BAD_REQUEST)

    if moveMode == "LIN":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        publisher_lin.publish(msg)
    elif moveMode =="Joint":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        publisher_joint.publish(msg)
    elif moveMode == "PTP":
        msg = Float64MultiArray()
        msg.data = [float(j) for j in dataArray] 
        publisher_work.publish(msg)
    
        return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0023(request):
    print("abort")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0024(request):
    print("home")

    return Response(status=status.HTTP_204_NO_CONTENT)