from rest_framework.decorators import api_view
from std_msgs.msg import Float64MultiArray, String
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path
from django.db.models import F
from django.core.cache import cache
from api.views.plc_manager import get_plc_manager
from api.views.move import controller
from api.views.components import robotData
import time

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0008(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
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
    idPoint = cache.get("idPoint")
    idPath = cache.get("idPath")
    grip = cache.get("grip")
    print(idPoint, idPath, grip)
    return Response({
        "name": result,
        "idPoint": idPoint,
        "idPath": idPath,
        "grip": grip
    })

@api_view(['POST'])
def O0026(request):
    # try:
    data = request.data
    idPoint = data.get('idPoint')
    idPath = data.get("idPath")
    grip = data.get("grip")
    cache.set("idPoint", idPoint)
    cache.set("idPath", idPath)

    theta_dict = Point.objects.filter(point_id=idPoint, path_id=idPath).values('t1', 't2', 't3', 't4', 't5', 't6')[0]
    grip_dict = Point.objects.filter(point_id=idPoint, path_id=idPath).values('ee')[0]
    
    if grip == "SKIP":
        if theta_dict:
            keys = ['t1', 't2', 't3', 't4', 't5', 't6']
            theta = [theta_dict[f] for f in keys]
            plc_manager.move_joint_degree(theta)
            cache.set("grip", grip_dict["ee"])
            return Response({"success": "point", "grip": grip_dict['ee']}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)
    else:
        if grip == "GRIP":
            plc_manager.write_device_block(device_name=["M350"], values=[1])
            time.sleep(0.3)
            plc_manager.write_device_block(device_name=["M350"], values=[0])
            cache.set("grip", "SKIP")
            return Response({"success": "grip", "grip": "SKIP"}, status=status.HTTP_200_OK)
        else:
            plc_manager.write_device_block(device_name=["M351"], values=[1])
            time.sleep(0.3)
            plc_manager.write_device_block(device_name=["M351"], values=[0])
            cache.set("grip", "SKIP")
            return Response({"success": "release", "grip": "SKIP"}, status=status.HTTP_200_OK)
        


            
    # except Exception as e:
    #     return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

# @api_view(['POST'])
# def grip(request):
#     # try:
#     data = request.data
#     bool_grip = data.get("grip")

#     if bool_grip == "GRIP":
#         plc_manager.write_device_block(device_name=["M350"], values=[1])
#         time.sleep()
#         plc_manager.write_device_block(device_name=["M350"], values=[0])
#     elif bool_grip == "RELEASE":
#         plc_manager.write_device_block(device_name=["M351"], values=[1])
#         time.sleep(1)
#         plc_manager.write_device_block(device_name=["M351"], values=[0])

#     return Response({"success": True}, status=status.HTTP_200_OK)
        
    # except Exception as e:
    #     return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)