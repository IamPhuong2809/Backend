from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import Point, Path
from django.db.models import F

@api_view(['GET'])
def O0008(request):
    paths = Path.objects.all().values('path_id', 'name')  
    result = [{'id': p['path_id'], 'name': p['name']} for p in paths]
    return Response(result)

@api_view(['POST'])
def O0026(request):
    data = request.data
    idPoint = data.get('idPoint')
    idPath = data.get("idPath")
    stepMode = data.get("stepMode")
    print(idPath, idPoint, stepMode)

    return Response(status=status.HTTP_204_NO_CONTENT)