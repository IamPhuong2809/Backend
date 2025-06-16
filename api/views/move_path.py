from rest_framework.decorators import api_view
from std_msgs.msg import Float64MultiArray, String
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path
from django.db.models import F
from api.views.plc_manager import get_plc_manager
from api.views.move import controller
from api.views.components import robotData
from api.views.Kinematics import quaternion_ik
import math
import time

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0008(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
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
    return Response(result)

@api_view(['POST'])
def O0026(request):
    try:
        data = request.data
        idPoint = data.get('idPoint')
        idPath = data.get("idPath")
        stepMode = data.get("stepMode")
        position = Point.objects.filter(point_id=idPoint, path_id=idPath).values('x', 'y', 'z', 'roll', 'pitch', 'yaw')[0]
        grip = Point.objects.filter(point_id=idPoint, path_id=idPath).values('ee')[0]
        if not position:
            return Response({"error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

        joint = [0, 0, 0, 0, 0, 0]
        keys = ['t1', 't2', 't3', 't4', 't5', 't6']
        for i, key in enumerate(keys):
            joint[i] = robotData["jointCurrent"][key]

        xyzrpy = [position['x'], position['y'], position['z'], position['roll'], position['pitch'], position['yaw']]

        msg = Float64MultiArray()
        # success, theta, _ , _ , _ , _ = quaternion_ik(xyzrpy, joint)
        success = True
        if success:
            msg.data = [j for j in xyzrpy]
            controller.publisher_joint.publish(msg)
            return Response({"success": True, "grip": grip}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)

    except Exception as e:
        return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

@api_view(['POST'])
def grip(request):
    # try:
    data = request.data
    bool_grip = data.get("grip")
    print(bool_grip)

    if bool_grip == "GRIP":
        plc_manager.write_device_block(device_name=["M350"], values=[1])
        time.sleep(1)
        plc_manager.write_device_block(device_name=["M350"], values=[0])
    elif bool_grip == "RELEASE":
        plc_manager.write_device_block(device_name=["M351"], values=[1])
        time.sleep(1)
        plc_manager.write_device_block(device_name=["M351"], values=[0])

    return Response({"success": True}, status=status.HTTP_200_OK)
        
    # except Exception as e:
    #     return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)