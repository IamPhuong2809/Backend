import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/horizon/datn/Backend/ros2_ws/install/ammr_plc_node'
