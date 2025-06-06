from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path
from django.db.models import F
from api.views.plc_manager import get_plc_manager
from api.views.move import controller


plc_manager = get_plc_manager()

@api_view(['GET'])
def O0008(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
    return Response(result)

@api_view(['POST'])
def O0026(request):
    try:
        data = request.data
        idPoint = data.get('idPoint')
        idPath = data.get("idPath")
        stepMode = data.get("stepMode")
        position = Point.objects.filter(point_id=idPoint, path_id=idPath).values('x', 'y', 'z', 'roll', 'pitch', 'yaw')[0]
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