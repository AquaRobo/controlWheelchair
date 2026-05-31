import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from utils.Configurator import Configurator
from tf_transformations import quaternion_from_euler
from my_robot_interfaces.msg import Encoders
from control.services.OdometryEvaluator import OdometryEvaluator

_RPM_TO_RAD_S = math.tau / 60.0   # 2π / 60

# Tune this by logging raw RPM while the wheelchair is completely stationary.
# Set it just above the maximum noise value you observe.
_RPM_NOISE_FLOOR = 0.5             # RPM

class OdomHardwareNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self._logger = self.get_logger()
        self.robot_config = Configurator("control").fetchData(
            Configurator.WHEELCHAIR_CONFIG
        )
        self.rear_wheel_radius = self.robot_config['rear_wheels_radius']
        self.rear_wheel_separation = self.robot_config['rear_wheels_separation']

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.encoders_sub = self.create_subscription(
            Encoders, '/encoders', self._encoderCallback, 10
        )

        # --- Odometry message (invariant fields) ---
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        # Pose covariance — 6×6 row-major [x,y,z,roll,pitch,yaw]
        # z/roll/pitch are meaningless for a ground robot → set very high.
        # x/y and yaw reflect typical encoder uncertainty; tune after testing.
        self.odom_msg.pose.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,
        ]

        # Twist covariance — same layout: only vx and wz are meaningful
        self.odom_msg.twist.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e9,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e9,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e9,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e9,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-2,
        ]

        self.prev_time = self.get_clock().now()
        self._logger.info("OdomHardwareNode started.")

    # ------------------------------------------------------------------
    # Encoder callback — sole source of odometry integration
    # ------------------------------------------------------------------
    def _encoderCallback(self, msg: Encoders) -> None:
        now = self.get_clock().now()
        dt = now - self.prev_time

        if dt.nanoseconds <= 0:
            self._logger.warn(
                f"Non-positive dt: {dt.nanoseconds} ns — skipping."
            )
            return

        self.prev_time = now
        dt_s = dt.nanoseconds / 1e9

        # --- Dead-band: zero out hall-effect noise at rest ---
        left_rpm  = msg.left_speed  if abs(msg.left_speed)  > _RPM_NOISE_FLOOR else 0.0
        right_rpm = msg.right_speed if abs(msg.right_speed) > _RPM_NOISE_FLOOR else 0.0

        # RPM → rad/s (wheel angular velocity)
        omega_left  = left_rpm  * _RPM_TO_RAD_S
        omega_right = right_rpm * _RPM_TO_RAD_S

        # Angular displacement of each wheel over dt [rad]
        dp_left  = omega_left  * dt_s
        dp_right = omega_right * dt_s

        # Robot-level velocities [m/s, rad/s] and pose increments [m, rad]
        linear_vel, angular_vel = OdometryEvaluator.getRobotVelocities(
            omega_left, omega_right,
            self.rear_wheel_radius, self.rear_wheel_separation
        )
        d_s, d_theta = OdometryEvaluator.getPositionIncrement(
            dp_left, dp_right,
            self.rear_wheel_radius, self.rear_wheel_separation
        )

        # Integrate pose — update heading BEFORE projecting displacement
        # so x/y use the heading at the END of the arc, not the start.
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        # Capture ONE timestamp — reused for both odom msg and TF.
        # Two separate now() calls produce slightly different stamps,
        # which causes TF extrapolation errors and map corruption.
        stamp = self.get_clock().now().to_msg()

        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # --- Publish odometry ---
        self.odom_msg.header.stamp             = stamp
        self.odom_msg.pose.pose.position.x     = self.x
        self.odom_msg.pose.pose.position.y     = self.y
        self.odom_msg.pose.pose.orientation.x  = q[0]
        self.odom_msg.pose.pose.orientation.y  = q[1]
        self.odom_msg.pose.pose.orientation.z  = q[2]
        self.odom_msg.pose.pose.orientation.w  = q[3]
        self.odom_msg.twist.twist.linear.x     = linear_vel
        self.odom_msg.twist.twist.angular.z    = angular_vel  # no negation: sign comes from getRobotVelocities
        self.odom_pub.publish(self.odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomHardwareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()