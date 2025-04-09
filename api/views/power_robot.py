from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status

@api_view(['GET'])
def O0000(request):
    print(f"Robot off")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0001(request):
    print(f"Robot on")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0002(request):
    print(f"Reset")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0003(request):
    print(f"Abort")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0010(request):
    data = request.data
    print(data)

    return Response(status=status.HTTP_204_NO_CONTENT)