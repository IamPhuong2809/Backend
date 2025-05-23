# coding: UTF-8

import time
import platform
import threading
import math
import lib.device_model as deviceModel
from lib.data_processor.roles.jy901s_dataProcessor import JY901SDataProcessor
from lib.protocol_resolver.roles.protocol_485_resolver import Protocol485Resolver
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation as R

welcome = """
Welcome to the Wit-Motion sample program (Read Only)
"""

class Node_IMU(Node):
    def __init__(self, device):
        super().__init__('Node_imu')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.device = device

        # Gắn callback
        self.device.dataProcessor.onVarChanged.append(self.onUpdate)

        # Tạo luồng đọc dữ liệu
        self.read_thread = threading.Thread(target=self.LoopReadThread)
        self.read_thread.daemon = True
        self.read_thread.start()

    def onUpdate(self, deviceModel):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Acceleration (m/s^2)
        imu_msg.linear_acceleration.x = deviceModel.getDeviceData("accX")
        imu_msg.linear_acceleration.y = deviceModel.getDeviceData("accY")
        imu_msg.linear_acceleration.z = deviceModel.getDeviceData("accZ")

        # Gyroscope (rad/s)
        imu_msg.angular_velocity.x = deviceModel.getDeviceData("gyroX")
        imu_msg.angular_velocity.y = deviceModel.getDeviceData("gyroY")
        imu_msg.angular_velocity.z = deviceModel.getDeviceData("gyroZ")

        # Convert angle (Euler in degrees) to quaternion
        roll = math.radians(deviceModel.getDeviceData("angleX"))
        pitch = math.radians(deviceModel.getDeviceData("angleY"))
        yaw = math.radians(deviceModel.getDeviceData("angleZ"))

        q = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.publisher.publish(imu_msg)

    def LoopReadThread(self):
        while rclpy.ok():
            self.device.readReg(0x30, 41)  # Read sensor data
            time.sleep(0.02)  # 50Hz (adjust as needed)

def main(args=None):
    print(welcome)

    # Tạo device model
    device = deviceModel.DeviceModel(
        "MyJY901",
        Protocol485Resolver(),
        JY901SDataProcessor(),
        "51_0"
    )

    device.ADDR = 0x50  # Sensor address
    if platform.system().lower() == 'linux':
        device.serialConfig.portName = "/dev/ttyUSB1"
    else:
        device.serialConfig.portName = "COM82"
    device.serialConfig.baud = 9600
    device.openDevice()

    rclpy.init(args=args)
    node = Node_IMU(device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    device.closeDevice()
    print("Device closed.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
