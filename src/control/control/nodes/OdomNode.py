import math
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from utils.Configurator import Configurator
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
from control.services.OdometryEvaluator import OdometryEvaluator

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self._logger = self.get_logger()
        self.robot_config = Configurator("control").fetchData(Configurator.WHEELCHAIR_CONFIG)
        self.rear_wheel_radius = self.robot_config['rear_wheels_radius']
        self.rear_wheel_separation = self.robot_config['rear_wheels_separation']

        self.rear_left_wheel_prev_pos = 0.0
        self.rear_right_wheel_prev_pos = 0.0
        self.front_left_wheel_prev_pos = 0.0
        self.front_right_wheel_prev_pos = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self._jointCallback, 10)

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
        dp_rear_left = msg.position[joint_names_index['lb_1_joint']] - self.rear_left_wheel_prev_pos
        dp_rear_right = msg.position[joint_names_index['rb_1_joint']] - self.rear_right_wheel_prev_pos 
        dt = Time.from_msg(msg.header.stamp) - self.prev_time

        # guard against zero/negative dt
        if dt.nanoseconds <= 0.0:
            self._logger.warn(f"Non-positive dt: {dt}. Skipping update.")

        # Actualize the prev pose for the next iteration
        self.rear_left_wheel_prev_pos = msg.position[joint_names_index['lb_1_joint']]
        self.rear_right_wheel_prev_pos = msg.position[joint_names_index['rb_1_joint']]
        self.prev_time = Time.from_msg(msg.header.stamp)

        fi_rear_left, fi_rear_right = OdometryEvaluator.getRotationalSpeeds(dp_rear_left, dp_rear_right, dt.nanoseconds)
        linear_vel, angular_vel = OdometryEvaluator.getRobotVelocities(fi_rear_left, fi_rear_right, self.rear_wheel_radius, self.rear_wheel_separation)
        d_s, d_theta = OdometryEvaluator.getPositionIncrement(dp_rear_left, dp_rear_right, self.rear_wheel_radius, self.rear_wheel_separation)

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
    odom_node = OdomNode()
    try:
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()