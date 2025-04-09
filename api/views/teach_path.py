from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status

@api_view(['POST'])
def O0016(request):
    data = request.data
    parameter = data.get('data')
    idPoint = data.get('idPoint')
    idPath = data.get("idPath")
    print(parameter,idPath, idPoint)

    return Response(status=status.HTTP_204_NO_CONTENT)