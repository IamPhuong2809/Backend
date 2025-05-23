from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path
from django.db import transaction
from django.db.models import F

@api_view(['GET'])
def O0007(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
    return Response(result)

@api_view(['POST'])
def O0016(request):
    data = request.data
    try:
        parameter = data.get('data')
        idPoint = data.get('idPoint')
        idPath = data.get("idPath")

        motion, cont, stop, vel, acc, corner = parameter
        cont_bool = True if cont.upper() == 'TRUE' else False
        stop_bool = True if stop.upper() == 'TRUE' else False
        updated = Point.objects.filter(point_id=idPoint, path_id=idPath).update(
            motion=motion,
            cont=cont_bool,
            stop=stop_bool,
            vel=vel,
            acc=acc,
            corner=corner
        )

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False, "error": "ID not found"}, status=status.HTTP_404_NOT_FOUND)

    except Exception as e:
        return Response({"success": False, "error": str(e)}, status=status.HTTP_400_BAD_REQUEST)


def insert_path_id(id, id_target):
    if id == id_target:
        return

    with transaction.atomic():
        Path.objects.filter(path_id=id).update(path_id=-1)
        Point.objects.filter(path_id=id).update(path_id=-1) 

        if id < id_target:
            paths = Path.objects.filter(path_id__gt=id, path_id__lte=id_target).order_by('path_id')
            Point.objects.filter(path_id__gt=id, path_id__lte=id_target).update(path_id=F('path_id') - 1)
            for path in paths:
                path.path_id -= 1
                path.save()

        else:
            paths = Path.objects.filter(path_id__gte=id_target, path_id__lt=id).order_by('-path_id')
            Point.objects.filter(path_id__gte=id_target, path_id__lt=id).update(path_id=F('path_id') + 1)
            for path in paths:
                path.path_id += 1
                path.save()

        Path.objects.filter(path_id=-1).update(path_id=id_target)
        Point.objects.filter(path__path_id=-1).update(path_id=id_target)


@api_view(['POST', 'DELETE'])
def path_list(request):    
    if request.method == 'POST':
        data = request.data
        type_data = data.get("type")
        if type_data == "add":
            id = data.get("id")
            name = data.get("name")
            with transaction.atomic():
                paths = Path.objects.filter(path_id__gte=id).order_by('path_id')
                for path in paths:
                    path.path_id += 1
                    path.save()
                Point.objects.filter(path_id__gte=id).update(path_id=F('path_id') + 1)
            Path.objects.create(path_id=id, name=name)
        elif type_data == "rename":
            id = data.get("id")
            name = data.get("name")
            Path.objects.filter(path_id=id).update(name=name)
        elif type_data == "update":
            id = data.get("id")
            id_target = data.get("id_target")
            insert_path_id(id, id_target)
        
        return Response(status=status.HTTP_204_NO_CONTENT)

    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            Path.objects.all().delete()
        else:
            id = data.get("id")
            with transaction.atomic():
                Path.objects.filter(path_id=id).delete()
                paths = Path.objects.filter(path_id__gt=id).order_by('path_id')
                for path in paths:
                    path.path_id -= 1
                    path.save()
                Point.objects.filter(path_id__gt=id).update(path_id=F('path_id') - 1)

        return Response(status=status.HTTP_204_NO_CONTENT)
    
    return Response({"error": "Method not allowed"}, status=status.HTTP_405_METHOD_NOT_ALLOWED)

def insert_point_id(id_path, id, id_target):
    if id == id_target:
        return

    with transaction.atomic():
        Point.objects.filter(path_id=id_path, point_id=id).update(point_id=-1)

        if id < id_target:
            Point.objects.filter(path_id=id_path, point_id__gt=id, point_id__lte=id_target).update(point_id=F('point_id') - 1)
        else:
            Point.objects.filter(path_id=id_path, point_id__gte=id_target, point_id__lt=id).update(point_id=F('point_id') + 1)

        Point.objects.filter(path_id=id_path, point_id=-1).update(point_id=id_target)

@api_view(['POST', 'DELETE'])
def point_list(request):    
    if request.method == 'POST':
        data = request.data
        type_data = data.get("type")
        if type_data == "add":
            id_path = data.get("id_parent")
            id = data.get("id")
            name = data.get("name")
            Point.objects.filter(path_id=id_path, point_id__gte=id).update(point_id=F('point_id') + 1)
            Point.objects.create(path_id=id_path,point_id=id,name=name,x=0,y=0,z=0,roll=0,pitch=0,yaw=0,
                                 tool=0,figure=0,work=0,motion="LIN",cont=False,stop=False,vel=0,acc=0,corner=0)
        elif type_data == "rename":
            id_path = data.get("id_parent")
            id = data.get("id")
            name = data.get("name")
            Point.objects.filter(path_id=id_path, point_id=id).update(name=name)
        elif type_data == "update":
            id = data.get("id")
            id_target = data.get("id_target")
            id_path = data.get("id_parent")
            insert_point_id(id_path, id, id_target)
        elif type_data == "data":
            id_path = data.get("id_parent")
            id = data.get("id")
            data_point = Point.objects.filter(path_id=id_path, point_id=id).values('x', 'y', 'z', 'roll', 'pitch', 'yaw',
                                                                                 'tool', 'figure', 'work', 'motion',
                                                                                 'cont', 'stop', 'vel', 'acc', 'corner') 
            return Response(data_point)
        else:
            id = data.get("id")
            points = Point.objects.filter(path_id=id).values('point_id', 'name')  
            result = [{'id': p['point_id'], 'name': p['name']} for p in points]
            return Response(result)
        
        return Response(status=status.HTTP_204_NO_CONTENT)

    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            id_path = data.get("id_parent")
            Point.objects.filter(path_id=id_path).delete()
        else:
            id_path = data.get("id_parent")
            id = data.get("id")
            Point.objects.filter(path_id=id_path, point_id=id).delete()
            Point.objects.filter(path_id=id_path, point_id__gt=id).update(point_id=F('point_id') - 1)

        return Response(status=status.HTTP_204_NO_CONTENT)
    
    return Response({"error": "Method not allowed"}, status=status.HTTP_405_METHOD_NOT_ALLOWED)