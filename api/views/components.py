from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pymcprotocol
import math
from api.views.plc_manager import get_plc_manager
import numpy as np
from scipy.spatial.transform import Rotation as R
import warnings
warnings.filterwarnings("ignore", category=UserWarning)

plc_manager = get_plc_manager()

pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
joint_real = [0, 0, 0, 0, 0, 0]
med = [-90, 0, -45, -90, -90, 0]
robotData = {
    "Power":True,
    "S": False,
    "I": False,
    "AUX": False,
    "busy": False,
    "ee": True,
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
    # updateRobotData()
    return Response(robotData)

def updateRobotData():
    try:
        #Joint
        data_joint = plc_manager.read_random(dword_devices=pos_addrs_read)
        joint = [round(val / 100000.0, 2)  for val in data_joint[1]]
        for i in range(5):
            joint_real[i] = joint[i] + med[i]
        if joint and len(joint) >= 6:
            keys = ['t1', 't2', 't3', 't4', 't5', 't6']
            for i, key in enumerate(keys):
                robotData["jointCurrent"][key] = joint_real[i]
        #Position
        pos_rpy = ForwardKinematis(joint_real)
        keys = ["x", "y", "z", "rl", "pt", "yw"]
        for i, key in enumerate(keys):
            robotData["positionCurrent"][key] = round(float(pos_rpy[i]), 3)

        #Bit
        robotData["busy"] = plc_manager._read_block_internal(device_name="M650", size=1)[0]
        robotData["S"] = plc_manager._read_block_internal(device_name="M651", size=1)[0]
        robotData["error"] = plc_manager._read_block_internal(device_name="M652", size=1)[0]

    except Exception as e:
        print(e)


def ForwardKinematis(joint_deg, N=6):
    L1 = 385.9; L2 = 40; L3 = 500; L4 = 0.9; L5 = 417.37; L6 = 60
    joint = np.radians(joint_deg)
    DH = np.array([
        [   0,       0,     L1,       joint[0] ],
        [ -L2,   np.pi/2,    0,       joint[1] ],
        [  L3,       0,    -L4,       joint[2] ],
        [   0,  -np.pi/2,  -L5,       joint[3] ],
        [   0,   np.pi/2,    0,       joint[4] ],
        [   0,   np.pi/2,    0,       joint[5] ],
        [   0,       0,     L6,       0        ]
    ])

    A = np.zeros((N+1, 4, 4))
    for i in range(N+1):
        a, alpha, d, theta = DH[i]
        ca, sa = np.cos(alpha), np.sin(alpha)
        ct, st = np.cos(theta), np.sin(theta)
        A[i] = np.array([
            [ct, -st, 0, a],
            [st*ca, ct*ca, -sa, -sa*d],
            [st*sa, ct*sa,  ca,  ca*d],
            [0, 0, 0, 1]
        ])

    T = A[0]
    for i in range(1, N+1):
        T = T @ A[i]
    position = T[:3, 3]         
    rotation = T[:3, :3]        

    r = R.from_matrix(rotation)
    rpy = r.as_euler('zyx', degrees=True)
    roll, pitch, yaw = rpy[2], rpy[1], rpy[0] 

    return np.array([*position, roll, pitch, yaw])

    

