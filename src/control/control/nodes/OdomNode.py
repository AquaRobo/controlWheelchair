import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from utils.Configurator import Configurator
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from control.services.OdometryEvaluator import OdometryEvaluator

class OdomNode(Node):
    """Wheel odometry from joint states (simulation variant).

    Integrates rear-wheel joint positions into a 2D pose (x, y, theta) using the
    differential-drive model in OdometryEvaluator, with wheel radius/separation
    read from wheelchair_config.yaml.

    Subscriptions:
        joint_states (sensor_msgs/JointState): wheel joint positions; uses
            'lb_joint' and 'rb_joint' (rear left/right).

    Publications:
        odom (nav_msgs/Odometry): pose + twist in the 'odom' frame,
            child frame 'base_footprint', published at 20 Hz.

    TF broadcasts:
        odom -> base_footprint transform at 20 Hz.
    """

    def __init__(self):
        super().__init__('odom_node')
        self._logger = self.get_logger()
        self.robot_config = Configurator("control").fetchData(Configurator.WHEELCHAIR_CONFIG)
        self.rear_wheel_radius = self.robot_config['rear_wheels_radius']
        self.rear_wheel_separation = self.robot_config['rear_wheels_separation']
        self.rear_left_wheel_prev_pos = 0.0
        self.rear_right_wheel_prev_pos = 0.0
        self._have_prev_joint_state = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.q = quaternion_from_euler(0, 0, self.theta)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self._jointCallback, 10)
        self.publish_timer = self.create_timer(0.05, self._publishOdom)  # Publish at 20 Hz

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
        
    def _jointCallback(self, msg: JointState) -> None:
        joint_names_index = {n: i for i, n in enumerate(msg.name)}
        rear_left_pos = msg.position[joint_names_index['lb_joint']]
        rear_right_pos = msg.position[joint_names_index['rb_joint']]
        now = self.get_clock().now()

        if not self._have_prev_joint_state:
            self.rear_left_wheel_prev_pos = rear_left_pos
            self.rear_right_wheel_prev_pos = rear_right_pos
            self.prev_time = now
            self._have_prev_joint_state = True
            return

        dp_rear_left = rear_left_pos - self.rear_left_wheel_prev_pos
        dp_rear_right = rear_right_pos - self.rear_right_wheel_prev_pos
        dt = now - self.prev_time

        self._logger.info(f"now={now.nanoseconds} prev={self.prev_time.nanoseconds} dt={dt.nanoseconds}")
        # guard against zero/negative dt
        if dt.nanoseconds <= 0:
            self._logger.warn(f"Non-positive dt: {dt.nanoseconds} ns. Skipping update.")
            return

        # Actualize the prev pose for the next iteration
        self.rear_left_wheel_prev_pos = rear_left_pos
        self.rear_right_wheel_prev_pos = rear_right_pos
        self.prev_time = now

        fi_rear_left, fi_rear_right = OdometryEvaluator.getRotationalSpeeds(dp_rear_left, dp_rear_right, dt.nanoseconds)
        linear_vel, angular_vel = OdometryEvaluator.getRobotVelocities(fi_rear_left, fi_rear_right, self.rear_wheel_radius, self.rear_wheel_separation)
        d_s, d_theta = OdometryEvaluator.getPositionIncrement(dp_rear_left, dp_rear_right, self.rear_wheel_radius, self.rear_wheel_separation)

        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

        # Compose the odom message (published by timer at 20 Hz)
        self.q = quaternion_from_euler(0, 0, self.theta)
        stamp = self.get_clock().now().to_msg()
        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.x = self.q[0]
        self.odom_msg.pose.pose.orientation.y = self.q[1]
        self.odom_msg.pose.pose.orientation.z = self.q[2]
        self.odom_msg.pose.pose.orientation.w = self.q[3]
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.angular.z = -angular_vel

        # TF
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = self.q[0]
        self.transform_stamped.transform.rotation.y = self.q[1]
        self.transform_stamped.transform.rotation.z = self.q[2]
        self.transform_stamped.transform.rotation.w = self.q[3]
        self.transform_stamped.header.stamp = stamp
        
    def _publishOdom(self) -> None:
        self.odom_pub.publish(self.odom_msg)
        self.br.sendTransform(self.transform_stamped)

        

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()
    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        odom_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()