from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pymcprotocol
import math
from api.views.plc_manager import get_plc_manager

plc_manager = get_plc_manager()

pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
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
    "positionCurrent": {"x": 0, "y": 1, "z": 2, "rl": 3, "pt": 4, "yw": 5},
    "jointCurrent": {"t1": 6, "t2": 7, "t3": 8, "t4": 9, "t5": 10, "t6": 11},
}

@api_view(['GET'])
def EMG(request):
    print("Emergency")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def I1001(request):
    updateRobotData()
    return Response(robotData)

def updateRobotData():
    try:
        #Joint
        data_pos = plc_manager.read_random(dword_devices=pos_addrs_read)
        position = [round(val / 100000.0, 2)  for val in data_pos[1]]
        if position and len(position) >= 6:
            keys = ['t1', 't2', 't3', 't4', 't5', 't6']
            for i, key in enumerate(keys):
                robotData["jointCurrent"][key] = position[i]
    except:
        pass
    

