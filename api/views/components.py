from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

robotData = {
    "Power":True,
    "S": False,
    "I": True,
    "AUX": True,
    "busy": False,
    "ee": False,
    "abort":False,
    "error": False,
    "override": 10,
    "tool": 5,
    "work": 2,
    "positionCurrent": {"x": 10, "y": 210, "z": 0, "rl": 3.5, "pt": 0, "yw": 0},
    "jointCurrent": {"t1": 0, "t2": 0, "t3": 0, "t4": 0, "t5": 0, "t6": 0},
}

@api_view(['GET'])
def EMG(request):
    print("Emergency")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def I1001(request):

    return Response(robotData)

def updateRobotData(request):
    global robotData
    # data = request.data
    # for key, value in data.items():
    #     if key in robot_state:
    #         robot_state[key] = value  # Cập nhật giá trị
    # return Response({"message": "Updated", "robot_state": robot_state})
