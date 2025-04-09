from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status

@api_view(['POST'])
def O0026(request):
    data = request.data
    idPoint = data.get('idPoint')
    idPath = data.get("idPath")
    print(idPath, idPoint)

    return Response(status=status.HTTP_204_NO_CONTENT)