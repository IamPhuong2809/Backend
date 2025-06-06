from rest_framework_simplejwt.tokens import RefreshToken
from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
from django.contrib.auth.hashers import check_password
from api.models import Accounts

@api_view(['POST'])
def Login(request):
    data = request.data
    username = data.get("username")
    password = data.get("password")
    if not username or not password:
        return Response({"message": "Missing username or password"}, status=status.HTTP_400_BAD_REQUEST)

    try:
        user = Accounts.objects.get(username=username)
        if not check_password(password, user.password):
            return Response({"message": "Invalid credentials"}, status=status.HTTP_401_UNAUTHORIZED)
        
        # Tạo JWT token
        refresh = RefreshToken.for_user(user)
        access_token = str(refresh.access_token)

        return Response({
            "message": "success",
            "token": access_token, 
            "fullname": user.fullname,
            "gmail": user.gmail,
        }, status=status.HTTP_200_OK)

    except Accounts.DoesNotExist:
        return Response({"message": "Invalid credentials"}, status=status.HTTP_401_UNAUTHORIZED)