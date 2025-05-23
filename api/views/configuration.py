from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from api.models import LoadData, LoadName

#region data type
# Data_Requests = [
#     {
#         "x": 10.00,
#         "y": 20.00,
#         "z": 30.00,
#         "mass": 4.00,
#         "jx": 50.00,
#         "jxy": 60.00,   
#         "jxz": 70.00,
#         "jyx": 80.00,
#         "jy": 90.00,
#         "jyz": 0.00,
#         "jzx": 0.00,
#         "jzy": 0.00,
#         "jz": 0.00
#     },
#     {
#         "x": 15.00,
#         "y": 25.00,
#         "z": 35.00,
#         "mass": 5.00,
#         "jx": 55.00,
#         "jxy": 65.00,   
#         "jxz": 75.00,
#         "jyx": 85.00,
#         "jy": 95.00,
#         "jyz": 5.00,
#         "jzx": 5.00,
#         "jzy": 5.00,
#         "jz": 5.00
#     },
#     {   
#         "x": 20.00,
#         "y": 30.00,
#         "z": 40.00,
#         "mass": 5.40,
#         "jx": 60.00,
#         "jxy": 70.00,   
#         "jxz": 80.00,
#         "jyx": 90.00,
#         "jy": 100.00,
#         "jyz": 10.00,
#         "jzx": 10.00,
#         "jzy": 10.00,
#         "jz": 10.00
#     }
# ]
# name_list = ["Load 1", "Load 2", "Load 3"]
#endregion

@api_view(['POST'])
def O0005(request):
    try:
        idLoad = int(request.data)  # Lấy index từ request
        if idLoad < 0 or idLoad >= 3:
            return Response({"error": "Invalid index"}, status=400)
        
        name_list = list(LoadName.objects.order_by("id").values_list("name", flat=True))
        data_list = LoadData.objects.filter(id=idLoad+1).values(
            "x", "y", "z", "mass", "jx", "jxy", "jxz", 
            "jyx", "jy", "jyz", "jzx", "jzy", "jz"
        ).first()
        
        response_data = {
            "dataLoad": data_list,  # Lấy dữ liệu theo ID
            "nameLoad": name_list,  # Trả về danh sách tên
        }

        return Response(response_data)
    
    except (ValueError, KeyError, TypeError) as e:
        return Response({"error": str(e)}, status=400)

@api_view(['POST'])
def O0013(request):
    data = request.data
    dataLoad = data.get('dataLoad')
    nameLoad = data.get('nameLoad')
    idLoad = data.get("id")
    
    LoadData.objects.filter(id=(idLoad+1)).update(**dataLoad)
    LoadName.objects.filter(id=(idLoad+1)).update(name=nameLoad)


    return Response(status=status.HTTP_204_NO_CONTENT)

