from rest_framework.decorators import api_view
from rest_framework.response import Response
from rest_framework import status
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
import math
import threading
from api.models import Point, Global
from api.views.plc_manager import get_plc_manager
from scipy.spatial.transform import Rotation as R
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

plc_manager = get_plc_manager()

pos_addrs_read = [f"D{addr}" for addr in range(1000, 1028, 3)]
robotData = {
    "Power": True,
    "S": True,
    "I": False,
    "AUX": False,
    "busy": False,
    "ee": True,
    "abort": False,
    "error": False,
    "override": 100,
    "tool": 0,
    "work": 0,
    "positionCurrent": {"x": 0, "y": 0, "z": 0, "rl": 0, "pt": 0, "yw": 0},
    "jointCurrent": {"t1": 0, "t2": 0, "t3": 0, "t4": 0, "t5": 0, "t6": 0},
}

# --- ROS2 Node TF Listener ---
class TFPositionListener(Node):
    def __init__(self):
        super().__init__('tf_position_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.lookup_transform)  # 5 Hz

    def lookup_transform(self):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'base_link',         
                'Gripper',           
                Time())             
            pos = trans.transform.translation
            q = trans.transform.rotation

            rotation = R.from_quat([q.x, q.y, q.z, q.w])
            rx, ry, rz = rotation.as_euler('xyz', degrees=False)

            robotData["positionCurrent"] = {
                "x": round(pos.x * 1000, 3),
                "y": round(pos.y * 1000, 3),
                "z": round(pos.z * 1000, 3),
                "rl": round(math.degrees(rx), 3),
                "pt": round(math.degrees(ry), 3),
                "yw": round(math.degrees(rz), 3),
            }

            robotData["busy"], robotData["S"], robotData["error"] = plc_manager.read_device_block(device_name="M650", size=3)

        except Exception as e:
            self.get_logger().warn(f"Không lấy được TF: {str(e)}")

# --- Start ROS Node in background thread ---
def start_ros_node():
    rclpy.init()
    node = TFPositionListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

ros_thread = threading.Thread(target=start_ros_node, daemon=True)
ros_thread.start()

# --- Django REST API views ---
@api_view(['GET'])
def EMG(request):
    plc_manager.write_device_block(device_name=["M208"], values=[1])
    return Response(status=status.HTTP_204_NO_CONTENT)

@api_view(['POST'])
def copy(request):
    data = request.data
    type_data = data.get("type")

    idCopy = data.get("idCopy")
    id = data.get("id")
    idPathCopy = data.get("idPathCopy")
    idPath = data.get("idPath")

    if type_data == "Global":
        theta_dict = Global.objects.filter(point_id=id).values('t1', 't2', 't3', 't4', 't5', 't6')[0]
        if idPathCopy == -1:
            updated = Global.objects.filter(point_id=idCopy).update(**theta_dict)
        else:
            updated = Point.objects.filter(path_id=idPathCopy, point_id=idCopy).update(**theta_dict)
    else:
        theta_dict = Point.objects.filter(path_id=idPath, point_id=id).values('t1', 't2', 't3', 't4', 't5', 't6')[0]
        if idPathCopy == -1:
            updated = Global.objects.filter(point_id=idCopy).update(**theta_dict)
        else:
            updated = Point.objects.filter(path_id=idPathCopy, point_id=idCopy).update(**theta_dict)

    if updated:
        return Response({"success": True}, status=status.HTTP_200_OK)
    else:
        return Response({"success": False}, status=status.HTTP_200_OK)

@api_view(['GET'])
def I1001(request):
    updateRobotData()
    return Response(robotData)

def updateRobotData():
    try:
        #Joint
        data_joint = plc_manager.read_random(dword_devices=pos_addrs_read)
        joint = [val / 100000.0  for val in data_joint[1]]
        if joint and len(joint) >= 6:
            keys = ['t1', 't2', 't3', 't4', 't5', 't6']
            for i, key in enumerate(keys):
                robotData["jointCurrent"][key] = round(joint[i], 3)
        #Bit
        robotData["busy"], robotData["S"], robotData["error"] = plc_manager.read_device_block(device_name="M650", size=3)
    except Exception as e:
        print(e)