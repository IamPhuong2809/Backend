from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from ammr_moveit_controller.srv import MoveRobot
from geometry_msgs.msg import PoseStamped
from api.models import Map, GoalPose, Aruco
from django.db.models import Max
import math
import threading
import time
import numpy as np
from api.views.plc_manager import get_plc_manager
from api.views.components import robotData
from transforms3d.euler import euler2quat
from fastapi import WebSocket

@api_view(['POST'])
def position_mobile(request):
    data = request.data
    id = data.get("id")
    id_site = data.get("id_site")
    try:
        map_obj = Map.objects.get(site_id=id_site, map_id=id)
    except Map.DoesNotExist:
        return Response({'error': 'Map not found'}, status=status.HTTP_404_NOT_FOUND)

    goals = GoalPose.objects.filter(map=map_obj).values('name','goal_id')
    result = [{'id': p['goal_id'], 'name': p['name']} for p in goals]
    return Response(result, status=status.HTTP_200_OK)

@api_view(['GET'])
def aruco_id(request):
    aruco_id = Aruco.objects.all().values('id_aruco')  
    result = [{'id': p['id_aruco']} for p in aruco_id   ]
    return Response(result)

@api_view(['POST'])
def save_action(request):
    data = request.data
    id = data.get("id")
    id_site = data.get("id_site")
    try:
        map_obj = Map.objects.get(site_id=id_site, map_id=id)
    except Map.DoesNotExist:
        return Response({'error': 'Map not found'}, status=status.HTTP_404_NOT_FOUND)

    goals = GoalPose.objects.filter(map=map_obj).values('name','goal_id')
    result = [{'id': p['goal_id'], 'name': p['name']} for p in goals]
    return Response(result, status=status.HTTP_200_OK)

@app.websocket("/status")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        await websocket.send_json({"status": "done"})
        await asyncio.sleep(2)
