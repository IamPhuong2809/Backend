from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path, Aruco
from django.db import transaction
from django.db.models import F
from api.views.plc_manager import get_plc_manager
from api.views.components import robotData
from api.views.Kinematics import ForwardKinematics

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0007(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
    plc_manager.rising_pulse(device_name=["M108"])
    jog_addrs_write = [f"D{addr}" for addr in range(2500, 2513, 2)]
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
def O0016(request):
    data = request.data
    try:
        parameter = data.get('data')
        idPoint = data.get('idPoint')
        idPath = data.get("idPath")

        motion, ee, stop, vel, acc, corner = parameter
        stop_bool = True if stop.upper() == 'TRUE' else False
        if motion == "P&P":
            updated = True
            pos = data.get("pos")
            motion, task, id, _, _, _ = parameter
            id = 0 if stop.upper() == 'FALSE' else int(id)
            point = Point.objects.get(point_id=idPoint, path_id=idPath)
            has_aruco = Aruco.objects.filter(point=point).exists()
            if has_aruco:
                return Response({"success": False, "error": "This point has a aruco id. Please choose another point to save"}, status=status.HTTP_200_OK)
            else:
                aruco, created = Aruco.objects.update_or_create(
                    point=point,
                    id_aruco=id,
                    defaults={
                        "task": task,
                        "x": pos[0], "y": pos[1], "z": pos[2],
                        "roll": pos[3], "pitch": pos[4], "yaw": pos[5]
                    }
                )
                if aruco or created:
                    updated = True
                else:
                    updated = False
        else:
            motion, ee, stop, vel, acc, corner = parameter
            stop_bool = True if stop.upper() == 'TRUE' else False
            updated = Point.objects.filter(point_id=idPoint, path_id=idPath).update(
                motion=motion,
                ee=ee,
                stop=stop_bool,
                vel=vel,
                acc=acc,
                corner=corner
            )

        if updated:  
            return Response({"success": True}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False, "error": "ID not found"}, status=status.HTTP_200_OK)

    except Exception as e:
        return Response({"success": False, "error": str(e)}, status=status.HTTP_400_BAD_REQUEST)
    
@api_view(['POST'])
def O0017(request):
    data = request.data
    try:
        idPoint = data.get('idPoint')
        idPath = data.get("idPath")

        theta_dict = Point.objects.filter(point_id=idPoint, path_id=idPath).values('t1', 't2', 't3', 't4', 't5', 't6')[0]
        grip = Point.objects.filter(point_id=idPoint, path_id=idPath).values('ee')[0]
        
        keys = ['t1', 't2', 't3', 't4', 't5', 't6']
        if theta_dict:
            theta = [theta_dict[f] for f in keys]
            success = True
        else:
            success = False
        if success:
            plc_manager.move_joint_degree(theta)
            return Response({"success": True, "grip": grip}, status=status.HTTP_200_OK)
        else:
            return Response({"success": False}, status=status.HTTP_200_OK)


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
            updated = Path.objects.create(path_id=id, name=name)
        elif type_data == "rename":
            id = data.get("id")
            name = data.get("name")
            updated = Path.objects.filter(path_id=id).update(name=name)
        elif type_data == "update":
            id = data.get("id")
            id_target = data.get("id_target")
            insert_path_id(id, id_target)
            updated = True
    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            updated = Path.objects.all().delete()
        else:
            id = data.get("id")
            with transaction.atomic():
                updated = Path.objects.filter(path_id=id).delete()
                paths = Path.objects.filter(path_id__gt=id).order_by('path_id')
                for path in paths:
                    path.path_id -= 1
                    path.save()
                Point.objects.filter(path_id__gt=id).update(path_id=F('path_id') - 1)

    if updated:  
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)
    
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
            updated = Point.objects.create(path_id=id_path,point_id=id,name=name,t1=0,t2=0,t3=0,t4=0,t5=0,t6=0,
                                 tool=0,figure=0,work=0,motion="LIN",ee="SKIP",stop=False,vel=0,acc=0,corner=0)
        elif type_data == "rename":
            id_path = data.get("id_parent")
            id = data.get("id")
            name = data.get("name")
            updated = Point.objects.filter(path_id=id_path, point_id=id).update(name=name)
        elif type_data == "update":
            id = data.get("id")
            id_target = data.get("id_target")
            id_path = data.get("id_parent")
            insert_point_id(id_path, id, id_target)
            updated = True
        elif type_data == "data":
            id_path = data.get("id_parent")
            id = data.get("id")

            data_point = Point.objects.filter(path_id=id_path, point_id=id).values('t1', 't2', 't3', 't4', 't5', 't6',
                                                                                 'tool', 'figure', 'work', 'motion',
                                                                                 'ee', 'stop', 'vel', 'acc', 'corner')[0]
            theta = [data_point['t1'] - 90, data_point['t2'], data_point['t3'] - 45, data_point['t4'] - 90, data_point['t5'] - 90, data_point['t6']]
            result = ForwardKinematics(theta)
            keys = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            for k, v in zip(keys, result):
                data_point[k] = v

            point = Point.objects.get(path_id=id_path, point_id=id)
            aruco = Aruco.objects.filter(point=point).first()
            if aruco:
                data_point['aruco'] = {
                    'id_aruco': aruco.id_aruco,
                    'task': aruco.task,
                    'x': aruco.x,
                    'y': aruco.y,
                    'z': aruco.z,
                    'roll': aruco.roll,
                    'pitch': aruco.pitch,
                    'yaw': aruco.yaw
                }
            else:
                data_point['aruco'] = None
            return Response(data_point)
        else:
            id = data.get("id")
            points = Point.objects.filter(path_id=id).values('point_id', 'name')  
            result = [{'id': p['point_id'], 'name': p['name']} for p in points]
            return Response(result)
    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            id_path = data.get("id_parent")
            updated = Point.objects.filter(path_id=id_path).delete()
        else:
            id_path = data.get("id_parent")
            id = data.get("id")
            updated = Point.objects.filter(path_id=id_path, point_id=id).delete()
            Point.objects.filter(path_id=id_path, point_id__gt=id).update(point_id=F('point_id') - 1)

    if updated:  
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)
    