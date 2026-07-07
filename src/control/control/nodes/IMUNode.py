import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from control.services.IMU import IMU

class IMUNode(Node):
    """Publishes IMU data received from the ESP over SPI.

    Polls the IMU service (which unpacks 10 floats from the ESP: quaternion,
    linear acceleration, angular velocity) at 10 Hz and republishes it as a
    standard ROS Imu message stamped in the 'imu_link' frame.

    Publications:
        imu (sensor_msgs/Imu): orientation, angular velocity and linear
            acceleration at 10 Hz.
    """

    def __init__(self):
        super().__init__('imu_node')
        self._logger = self.get_logger()
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        self.imu = IMU()
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"
        self.timer = self.create_timer(0.1, self.run)  # Publish at 10 Hz

    def run(self):
        self.imu.update()
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        orientation = self.imu.getOrientation()
        angular_velocity = self.imu.getAngularVelocity()
        linear_acceleration = self.imu.getLinearAcceleration()

        self.imu_msg.orientation.x = orientation[0]
        self.imu_msg.orientation.y = orientation[1]
        self.imu_msg.orientation.z = orientation[2]
        self.imu_msg.orientation.w = orientation[3]

        self.imu_msg.angular_velocity.x = angular_velocity[0]
        self.imu_msg.angular_velocity.y = angular_velocity[1]
        self.imu_msg.angular_velocity.z = angular_velocity[2]

        self.imu_msg.linear_acceleration.x = linear_acceleration[0]
        self.imu_msg.linear_acceleration.y = linear_acceleration[1]
        self.imu_msg.linear_acceleration.z = linear_acceleration[2]

        self.imu_pub.publish(self.imu_msg)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
