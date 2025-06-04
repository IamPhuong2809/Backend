from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.views.plc_manager import get_plc_manager

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0000(request):
    success = plc_manager.write_random(
        word_devices=["M0"],
        word_values=[1]
    )

    if success:
        print("[PLC] Robot power off command sent")
    else:
        print("[PLC] Failed to send robot power off command")

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