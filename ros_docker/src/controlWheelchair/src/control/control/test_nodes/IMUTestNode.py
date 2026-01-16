import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUTestNode(Node):
    def __init__(self):
        super().__init__('imu_test_node')
        self._logger = self.get_logger()
        self.imu_msg = Imu()
        self.orientation = None
        self.Yaw = 0.0
        self.Pitch = 0.0
        self.Roll = 0.0
        self.create_subscription(Imu, '/imu', self._imu_callback, 10)

    def _imu_callback(self, msg: Imu):
        self.orientation = msg.orientation
        self.__eularFromQuaternion()
        self._logger.info(f"Yaw: {math.degrees(self.Yaw):.2f}, Pitch: {math.degrees(self.Pitch):.2f}, Roll: {math.degrees(self.Roll):.2f}")

    def __eularFromQuaternion(self):
        if self.orientation is not None:
            qx = self.orientation.x
            qy = self.orientation.y
            qz = self.orientation.z
            qw = self.orientation.w

            # Yaw (Z-axis rotation)
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            self.Yaw = math.atan2(siny_cosp, cosy_cosp)

            # Pitch (Y-axis rotation)
            sinp = 2.0 * (qw * qy - qz * qx)
            self.Pitch = math.asin(sinp)

            # Roll (X-axis rotation)
            sinr_cosp = 2.0 * (qw * qx + qy * qz)
            cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
            self.Roll = math.atan2(sinr_cosp, cosr_cosp)

def main(args=None):
    rclpy.init(args=args)
    imu_test_node = IMUTestNode()
    rclpy.spin(imu_test_node)
    imu_test_node.destroy_node()
    rclpy.shutdown()


