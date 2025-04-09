import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/phuongnguyen/DATN/Backend/ros2_ws/install/robot_subscriber'
