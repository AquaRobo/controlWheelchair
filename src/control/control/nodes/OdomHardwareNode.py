import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from utils.Configurator import Configurator
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from my_robot_interfaces.msg import Encoders
from control.services.OdometryEvaluator import OdometryEvaluator

_RPM_TO_RAD_S = math.tau / 60.0  # 2π / 60

class OdomHardwareNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self._logger = self.get_logger()
        self.robot_config = Configurator("control").fetchData(Configurator.WHEELCHAIR_CONFIG)
        self.rear_wheel_radius = self.robot_config['rear_wheels_radius']
        self.rear_wheel_separation = self.robot_config['rear_wheels_separation']

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.imu_yaw_rate: float | None = None

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.encoders_sub = self.create_subscription(Encoders, '/encoders', self._encoderCallback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self._imuCallback, 10)

        # Fill the Odometry message with invariant parameters
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        self.br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

        self.prev_time = self.get_clock().now()

    def _imuCallback(self, msg: Imu) -> None:
        self.imu_yaw_rate = msg.angular_velocity.z

    def _encoderCallback(self, msg: Encoders) -> None:
        now = self.get_clock().now()
        dt = now - self.prev_time

        # guard against zero/negative dt
        if dt.nanoseconds <= 0:
            self._logger.warn(f"Non-positive dt: {dt.nanoseconds} ns. Skipping update.")
            return

        self.prev_time = now
        dt_s = dt.nanoseconds / 1e9

        # Convert RPM → rad/s, then to angular displacement (rad) over dt
        omega_left = msg.left_speed * _RPM_TO_RAD_S
        omega_right = msg.right_speed * _RPM_TO_RAD_S
        dp_left = omega_left * dt_s
        dp_right = omega_right * dt_s

        linear_vel, _ = OdometryEvaluator.getRobotVelocities(
            omega_left, omega_right, self.rear_wheel_radius, self.rear_wheel_separation
        )
        d_s, d_theta_enc = OdometryEvaluator.getPositionIncrement(
            dp_left, dp_right, self.rear_wheel_radius, self.rear_wheel_separation
        )

        # Use IMU yaw rate for heading if available, otherwise fall back to encoders
        if self.imu_yaw_rate is not None:
            d_theta = self.imu_yaw_rate * dt_s
            angular_vel = self.imu_yaw_rate
        else:
            d_theta = d_theta_enc
            _, angular_vel = OdometryEvaluator.getRobotVelocities(
                omega_left, omega_right, self.rear_wheel_radius, self.rear_wheel_separation
            )

        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.angular.z = -angular_vel
        self.odom_pub.publish(self.odom_msg)

        # TF
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    odom_hardware_node = OdomHardwareNode()
    try:
        rclpy.spin(odom_hardware_node)
    except KeyboardInterrupt:
        pass
    finally:
        odom_hardware_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
