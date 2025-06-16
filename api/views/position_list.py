from rest_framework.decorators import api_view
from rest_framework.response import Response
from std_msgs.msg import Float64MultiArray, String
from rest_framework import status
from api.models import Global
from django.db import transaction
from api.views.plc_manager import get_plc_manager
from django.db.models import F
from api.views.move import controller
from api.views.components import robotData
from api.views.Kinematics import quaternion_ik, ForwardKinematics
import math

plc_manager = get_plc_manager()


@api_view(['GET'])
def O0006(request):
    points = Global.objects.all().values('point_id', 'name')  
    result = [{'id': p['point_id'], 'name': p['name']} for p in points]
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
def O0014(request):
    try:
        data = request.data
        id = data.get("id")
        position = Global.objects.filter(point_id=id).values('x', 'y', 'z', 'roll', 'pitch', 'yaw')[0]
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
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)

    except Exception as e:
        return Response({"error": str(e)}, status=status.HTTP_400_BAD_REQUEST)

@api_view(['GET'])
def O0015(request):

    return Response({"success": True}, status=status.HTTP_200_OK)


def insert_point_id(id, id_target):
    if id == id_target:
        return

    with transaction.atomic():
        Global.objects.filter(point_id=id).update(point_id=-1)

        if id < id_target:
            Global.objects.filter(point_id__gt=id, point_id__lte=id_target).update(point_id=F('point_id') - 1)
        else:
            Global.objects.filter(point_id__gte=id_target, point_id__lt=id).update(point_id=F('point_id') + 1)

        Global.objects.filter(point_id=-1).update(point_id=id_target)

@api_view(['POST', 'DELETE'])
def global_list(request):    
    if request.method == 'POST':
        data = request.data
        type_data = data.get("type")
        if type_data == "add":
            id = data.get("id")
            name = data.get("name")
            Global.objects.filter(point_id__gte=id).update(point_id=F('point_id') + 1)
            updated = Global.objects.create(point_id=id, name=name, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, tool=0, figure=0, work=0)
        elif type_data == "rename":
            id = data.get("id")
            name = data.get("name")
            updated = Global.objects.filter(point_id=id).update(name=name)
        elif type_data == "update":
            id = data.get("id")
            id_target = data.get("id_target")
            insert_point_id(id, id_target)
            updated = True
        else:
            id = data.get("id")
            position = Global.objects.filter(point_id=id).values('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'figure') 
            # input_xyzrpy = [position[0]['x'], position[0]['y'], position[0]['z'], position[0]['roll'], position[0]['pitch'], position[0]['yaw']]
            # result = ForwardKinematics(input_xyzrpy)
            # print(result)
            # new_position = {
            #     'x': result[0],
            #     'y': result[1],
            #     'z': result[2],
            #     'roll': result[3],
            #     'pitch': result[4],
            #     'yaw': result[5],
            #     'figure': position.get('figure')
            # }

            return Response(position[0])
        
        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)

    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            updated = Global.objects.all().delete()
        else:
            id = data.get("id")
            Global.objects.filter(point_id=id).delete()
            updated = Global.objects.filter(point_id__gt=id).update(point_id=F('point_id') - 1)

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)
    
    return Response({"error": "Method not allowed"}, status=status.HTTP_405_METHOD_NOT_ALLOWED)