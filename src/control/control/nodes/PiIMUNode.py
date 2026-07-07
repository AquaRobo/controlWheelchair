import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from control.services.PiIMU import IMU


class PiIMUNode(Node):
    """Publishes IMU data from a BNO08x connected directly to the Pi over I2C.

    Alternative to IMUNode for setups where the IMU is wired to the Raspberry Pi
    instead of the ESP. Reads quaternion, gyro and accelerometer from the PiIMU
    service at 20 Hz and republishes them stamped in the 'imu_link' frame.

    Publications:
        imu (sensor_msgs/Imu): orientation, angular velocity and linear
            acceleration at 20 Hz.
    """

    def __init__(self):
        super().__init__('imu_node')
        self._logger = self.get_logger()
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.imu = IMU()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        self.timer = self.create_timer(0.05, self.run)  # Publish at 20 Hz

    def run(self):
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()

        # BNO08x returns quaternion as (i, j, k, real) -> maps to (x, y, z, w)
        quat_i, quat_j, quat_k, quat_real = self.imu.get_rotation()
        self.imu_msg.orientation.x = quat_i
        self.imu_msg.orientation.y = quat_j
        self.imu_msg.orientation.z = quat_k
        self.imu_msg.orientation.w = quat_real

        gyro_x, gyro_y, gyro_z = self.imu.get_gyro()
        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z

        accel_x, accel_y, accel_z = self.imu.get_acceleration()
        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z

        self.imu_pub.publish(self.imu_msg)


def main(args=None):
    rclpy.init(args=args)
    pi_imu_node = PiIMUNode()
    try:
        rclpy.spin(pi_imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        pi_imu_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
