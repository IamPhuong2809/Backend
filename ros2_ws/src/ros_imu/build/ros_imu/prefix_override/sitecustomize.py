import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/horizon/datn/Backend/ros2_ws/src/ros_imu/install/ros_imu'
