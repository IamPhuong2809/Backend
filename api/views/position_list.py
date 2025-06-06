from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Global
from django.db import transaction
from django.db.models import F
from api.views.move import controller

@api_view(['GET'])
def O0006(request):
    points = Global.objects.all().values('point_id', 'name')  
    result = [{'id': p['point_id'], 'name': p['name']} for p in points]
    return Response(result)

@api_view(['POST'])
def O0014(request):
    try:
        data = request.data
        id = data.get("id")
        position = Global.objects.filter(point_id=id).values('x', 'y', 'z', 'roll', 'pitch', 'yaw')[0]
        if not position:
            return Response({"error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

        msg = controller.create_pose_message(
            position["x"], position["y"], position["z"],
            position["roll"], position["pitch"], position["yaw"]
        )

        controller.publisher_work.publish(msg)
        return Response({"success": True}, status=status.HTTP_200_OK)

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