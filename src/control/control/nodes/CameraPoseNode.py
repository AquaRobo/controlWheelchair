#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.duration import Duration


class CameraPoseNode(Node):

    def __init__(self):
        super().__init__('camera_pose_node')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('source_frame', 'lsm36156_left_optical_frame')
        self.declare_parameter('tf_timeout', 0.5)

        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame = self.get_parameter('source_frame').value

        self._tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout').value
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.world_pub_ = self.create_publisher(
            PointStamped,
            '/base_link/object_position',
            10
        )

        self.create_subscription(
            PointStamped,
            '/camera/object_position',
            self._on_camera_point,
            10
        )

        self.get_logger().info(
            f'CameraPoseNode running: {self.source_frame} → {self.target_frame}'
        )

    def _on_camera_point(self, msg: PointStamped):

        if not msg.header.frame_id:
            msg.header.frame_id = self.source_frame

        try:
            base_point = self.tf_buffer.transform(
                msg,
                self.target_frame,
                timeout=self._tf_timeout
            )

        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'TF failed ({msg.header.frame_id} → {self.target_frame}): {exc}'
            )
            return

        self.world_pub_.publish(base_point)

        self.get_logger().debug(
            f'Camera: ({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}) '
            f'→ Base: ({base_point.point.x:.3f}, '
            f'{base_point.point.y:.3f}, {base_point.point.z:.3f})'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()