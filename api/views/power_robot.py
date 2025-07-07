from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import time
from api.views.plc_manager import get_plc_manager
from api.models import Home

plc_manager = get_plc_manager()

@api_view(['GET'])
def O0000(request):
    plc_manager.write_device_block(device_name=["M207"], values=[1])
    time.sleep(0.1)
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
    plc_manager.write_device_block(device_name=["M207"], values=[1])
    time.sleep(0.1)
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
    plc_manager.write_device_block(device_name=["M207"], values=[1])
    time.sleep(0.1)
    plc_manager.rising_pulse(device_name=["M115"])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0003(request):
    plc_manager.write_device_block(device_name=["M203"], values=[1])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0010(request):
    data = request.data
    Home.objects.create(t1=data[0], t2=data[1], t3=data[2], t4=data[3], t5=data[4], t6=data[5])

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def O0004(request):
    theta = Home.objects.all().values('id', 't1', 't2', 't3', 't4', 't5', 't6')

    return Response(theta)