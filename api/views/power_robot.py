from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import time
from api.views.plc_manager import get_plc_manager

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0000(request):
    plc_manager.write_device_block(device_name=["M108"], values=[1])
    time.sleep(0.02)
    plc_manager.write_device_block(device_name=["M108"], values=[0])

    success = plc_manager.write_device_block(
        device_name=["M30042"],
        values=[0]
    )

    if success:
        print("[PLC] Robot power off command sent")
    else:
        print("[PLC] Failed to send robot power off command")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0001(request):
    success = plc_manager.write_device_block(
        device_name=["M30042"],
        values=[1]
    )

    if success:
        print("[PLC] Robot power on command sent")
    else:
        print("[PLC] Failed to send robot power off command")

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