from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Map, Site
from django.db import transaction
from django.db.models import F
import os
import shutil
from api.views.plc_manager import get_plc_manager
import time

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0031(request):
    sites = Site.objects.all().values('site_id', 'name')  
    result = [{'id': p['site_id'], 'name': p['name']} for p in sites]
    return Response(result)

@api_view(['GET'])
def missions(request):
    plc_manager.rising_pulse(device_name=["M108"])
    plc_manager.rising_pulse(device_name=["M110"])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def record(request):
    plc_manager.rising_pulse(device_name=["M108"])
    plc_manager.rising_pulse(device_name=["M110"])

    
    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST', 'DELETE'])
def site_list(request):    
    if request.method == 'POST':
        data = request.data
        type_data = data.get("type")
        if type_data == "add":
            id = data.get("id")
            name = data.get("name")
            with transaction.atomic():
                sites = Site.objects.filter(site_id__gte=id).order_by('site_id')
                for site in sites:
                    site.site_id += 1
                    site.save()
                Map.objects.filter(site_id__gte=id).update(site_id=F('site_id') + 1)
            updated = Site.objects.create(site_id=id, name=name)
        elif type_data == "rename":
            id = data.get("id")
            name = data.get("name")
            updated = Site.objects.filter(site_id=id).update(name=name)
    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            updated = Site.objects.all().delete()
        else:
            id = data.get("id")
            with transaction.atomic():
                updated = Site.objects.filter(site_id=id).delete()
                sites = Site.objects.filter(site_id__gt=id).order_by('site_id')
                for site in sites:
                    site.site_id -= 1
                    site.save()
                Map.objects.filter(site_id__gt=id).update(site_id=F('site_id') - 1)
    
    if updated:  
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)

@api_view(['POST', 'DELETE'])
def map_list(request):    
    if request.method == 'POST':
        data = request.data
        type_data = data.get("type")
        if type_data == "add":
            id_site = data.get("id_parent")
            id = data.get("id")
            name = data.get("name")
            Map.objects.filter(site_id=id_site, map_id__gte=id).update(map_id=F('map_id') + 1)
            updated = Map.objects.create(site_id=id_site,map_id=id,name=name,new_file=True, name_file='')
        elif type_data == "rename":
            id_site = data.get("id_parent")
            id = data.get("id")
            name = data.get("name")
            updated = Map.objects.filter(site_id=id_site, map_id=id).update(name=name)
        elif type_data == "data":
            id_site = data.get("id_parent")
            id = data.get("id")
            data_map = Map.objects.filter(site_id=id_site, map_id=id).values('new_file', 'name_file') 
            return Response(data_map)
        else:
            id = data.get("id")
            maps = Map.objects.filter(site_id=id).values('map_id', 'name')  
            result = [{'id': p['map_id'], 'name': p['name']} for p in maps]
            return Response(result)
    elif request.method == 'DELETE':
        data = request.data
        delete_all = data.get("delete_all")
        if delete_all:
            id_site = data.get("id_parent")
            updated = Map.objects.filter(site_id=id_site).delete()
        else:
            id_site = data.get("id_parent")
            id = data.get("id")
            updated = Map.objects.filter(site_id=id_site, map_id=id).delete()
            Map.objects.filter(site_id=id_site, map_id__gt=id).update(map_id=F('map_id') - 1)

    if updated:  
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)
    
@api_view(['POST'])
def save_map(request):
    data = request.data
    file_name = data.get('fileName')
    id_site = data.get("id_parent")
    id = data.get("id")

    if not file_name:
        return Response({'status': 'error', 'message': 'Missing fileName'}, status=400)
    db_dir = os.path.expanduser('~/datn/database/')

    src = os.path.join(db_dir, 'rtabmap.db.back')
    dst = os.path.join(db_dir, file_name)

    if not os.path.exists(src):
        return Response({'status': 'error', 'message': 'rtabmap.db.back not found'}, status=404)

    # Copy file
    shutil.copy2(src, dst)

    updated = Map.objects.filter(site_id=id_site, map_id=id).update(new_file=False, name_file=file_name) 
    
    if updated:  
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_404_NOT_FOUND)