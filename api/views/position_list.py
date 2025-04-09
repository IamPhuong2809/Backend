from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status

@api_view(['POST'])
def O0005(request):
    try:
        idLoad = int(request.data)  # Lấy index từ request
        if idLoad < 0 or idLoad >= 3:
            return Response({"error": "Invalid index"}, status=400)
        
        # name_list = list(LoadName.objects.order_by("id").values_list("name", flat=True))
        # data_list = LoadData.objects.filter(id=idLoad+1).values(
        #     "x", "y", "z", "mass", "jx", "jxy", "jxz", 
        #     "jyx", "jy", "jyz", "jzx", "jzy", "jz"
        # ).first()
        # print(data_list)
        
        # response_data = {
        #     "dataLoad": data_list,  # Lấy dữ liệu theo ID
        #     "nameLoad": name_list,  # Trả về danh sách tên
        # }

        # return Response(response_data)
    
    except (ValueError, KeyError, TypeError) as e:
        return Response({"error": str(e)}, status=400)

@api_view(['GET'])
def O0014(request):
    print(f"Robot run")

    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['GET'])
def O0015(request):
    print(f"Robot abort")

    return Response(status=status.HTTP_204_NO_CONTENT)