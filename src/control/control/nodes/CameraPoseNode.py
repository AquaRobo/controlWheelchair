#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped
from my_robot_interfaces.msg import ObjectDetection


class CameraPoseNode(Node):
    """Transforms camera object detections into the robot base frame.

    Takes detections expressed in the camera optical frame and re-expresses
    their position in the target frame via TF2, so downstream consumers
    (e.g. ArmPoseNode grab targets) can work in base_link coordinates.
    Detections are dropped with a warning if the transform is unavailable.

    Parameters:
        target_frame (string, default 'base_link'): frame to transform into.
        source_frame (string, default 'lsm36156_left_optical_frame'): camera frame.
        tf_timeout (double, default 0.5): TF lookup timeout in seconds.

    Subscriptions:
        /object_detection (my_robot_interfaces/ObjectDetection): detection with
            object position in the camera frame.

    Publications:
        /base_link/object_detection (my_robot_interfaces/ObjectDetection):
            same detection with position re-expressed in target_frame.
    """

    def __init__(self):
        super().__init__('camera_pose_node')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'lsm36156_left_optical_frame')
        self.declare_parameter('tf_timeout', 0.5)

        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value

        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value

        self._tf_timeout = Duration(
            seconds=self.get_parameter(
                'tf_timeout').get_parameter_value().double_value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.publisher = self.create_publisher(
            ObjectDetection,
            '/base_link/object_detection',
            10
        )

        self.subscription = self.create_subscription(
            ObjectDetection,
            '/object_detection',
            self._on_detection,
            10
        )

        self.get_logger().info(
            f'CameraPoseNode running: '
            f'{self.source_frame} -> {self.target_frame}'
        )

    def _on_detection(self, msg: ObjectDetection):

        point = PointStamped()

        point.header = msg.header
        point.header.frame_id = self.source_frame

        point.point.x = msg.object_pose.x
        point.point.y = msg.object_pose.y
        point.point.z = msg.object_pose.z

        try:
            base_point = self.tf_buffer.transform(
                point,
                self.target_frame,
                timeout=self._tf_timeout
            )

        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'TF failed ({point.header.frame_id} -> '
                f'{self.target_frame}): {exc}'
            )
            return

        out_msg = copy.deepcopy(msg)

        out_msg.header.frame_id = self.target_frame
        out_msg.header.stamp = base_point.header.stamp

        out_msg.object_pose.x = base_point.point.x
        out_msg.object_pose.y = base_point.point.y
        out_msg.object_pose.z = base_point.point.z

        self.publisher.publish(out_msg)

        self.get_logger().debug(
            f'{msg.object_name}: '
            f'Camera ({point.point.x:.3f}, '
            f'{point.point.y:.3f}, '
            f'{point.point.z:.3f}) -> '
            f'Base ({base_point.point.x:.3f}, '
            f'{base_point.point.y:.3f}, '
            f'{base_point.point.z:.3f})'
        )


def main(args=None):
    rclpy.init(args=args)

    node = CameraPoseNode()

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