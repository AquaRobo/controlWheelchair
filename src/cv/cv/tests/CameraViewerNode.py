#!/usr/bin/env python3
"""CameraViewerNode — development/debug tool.

Subscribes to all camera topics published by CameraStreamerNode and
renders them in OpenCV windows.  One window is opened per topic.
Also subscribes to /object_detection and /room_identification and shows
their YOLO-annotated frames with overlaid labels.

Mono cameras:
    /{camera_name}/uncalibrated
    /{camera_name}/calibrated       (only when calibration != NONE)

Stereo cameras:
    /{camera_name}/left/uncalibrated
    /{camera_name}/left/calibrated  (only when calibration != NONE)
    /{camera_name}/right/uncalibrated
    /{camera_name}/right/calibrated (only when calibration != NONE)

Object detection:
    /object_detection               (annotated frame with pose overlay)

Room identification:
    /room_identification            (annotated frame with room + confidence)

Press  q  or  Esc  in any window to exit.

Usage
-----
    ros2 run cv camera_viewer_node
"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import ObjectDetection, RoomIdentification
from utils.Configurator import Configurator
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

_QOS_DEPTH = 10
_OD_WINDOW = 'object_detection | annotated'
_RI_WINDOW = 'room_identification | annotated'


def _has_calibration(cam_cfg: dict) -> bool:
    """Return True when a real calibration file is configured."""
    cal = cam_cfg.get('calibration', 'NONE')
    return str(cal).upper() != 'NONE'


class CameraViewerNode(Node):
    """Visualises calibrated and uncalibrated image topics in OpenCV windows."""

    def __init__(self) -> None:
        super().__init__('camera_viewer_node')
        self._log = self.get_logger()
        self._bridge = CvBridge()
        # window_title → latest numpy frame (or None)
        self._frames: dict[str, object] = {}

        self._setup_subscriptions()
        self._setup_object_detection_subscription()
        self._setup_room_identification_subscription()

    # ------------------------------------------------------------------
    # Setup

    def _setup_subscriptions(self) -> None:
        try:
            cameras_cfg = Configurator('cv').fetchData(Configurator.CAMERAS)
        except Exception as exc:
            self._log.error(f'CameraViewerNode: failed to load cameras.yaml: {exc}')
            return

        if not cameras_cfg:
            self._log.warning('CameraViewerNode: cameras.yaml is empty — no topics to subscribe to.')
            return

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        for cam_name, cam_cfg in cameras_cfg.items():
            is_stereo = cam_cfg.get('is_stereo', False)
            calibrated = _has_calibration(cam_cfg)

            if is_stereo:
                self._subscribe_stereo(cam_name, calibrated, image_qos)
            else:
                self._subscribe_mono(cam_name, calibrated, image_qos)

    def _setup_object_detection_subscription(self) -> None:
        self._frames[_OD_WINDOW] = None
        self.create_subscription(
            ObjectDetection,
            '/object_detection',
            self._on_detection,
            _QOS_DEPTH,
        )
        self._log.info('CameraViewerNode: subscribed to /object_detection')

    def _setup_room_identification_subscription(self) -> None:
        self._frames[_RI_WINDOW] = None
        self.create_subscription(
            RoomIdentification,
            '/room_identification',
            self._on_room_identification,
            _QOS_DEPTH,
        )
        self._log.info('CameraViewerNode: subscribed to /room_identification')

    def _subscribe_mono(self, cam_name: str, calibrated: bool, qos) -> None:
        self._add_subscription(f'/{cam_name}/uncalibrated', f'{cam_name} | uncalibrated', qos)
        if calibrated:
            self._add_subscription(f'/{cam_name}/calibrated', f'{cam_name} | calibrated', qos)

    def _subscribe_stereo(self, cam_name: str, calibrated: bool, qos) -> None:
        for side in ('left', 'right'):
            self._add_subscription(
                f'/{cam_name}/{side}/uncalibrated',
                f'{cam_name} | {side} | uncalibrated',
                qos,
            )
            if calibrated:
                self._add_subscription(
                    f'/{cam_name}/{side}/calibrated',
                    f'{cam_name} | {side} | calibrated',
                    qos,
                )

    def _add_subscription(self, topic: str, window_title: str, qos) -> None:
        self._frames[window_title] = None
        self.create_subscription(
            Image,
            topic,
            lambda msg, t=window_title: self._on_frame(msg, t),
            qos,
        )
        self._log.info(f'CameraViewerNode: subscribed to {topic}')

    # ------------------------------------------------------------------
    # Subscription callbacks

    def _on_frame(self, msg: Image, window_title: str) -> None:
        try:
            self._frames[window_title] = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._log.error(f'CameraViewerNode: conversion error for "{window_title}": {exc}')

    def _on_detection(self, msg: ObjectDetection) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg.annotated_frame, desired_encoding='bgr8')
        except Exception as exc:
            self._log.error(f'CameraViewerNode: annotated frame conversion error: {exc}')
            return

        p = msg.object_pose
        label = (
            f"{msg.object_name}  conf={msg.confidence:.2f}"
            f"  X={p.x:.3f}  Y={p.y:.3f}  Z={p.z:.3f} m"
        )
        cv2.putText(
            frame, label,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
            (0, 255, 0), 2, cv2.LINE_AA,
        )
        self._frames[_OD_WINDOW] = frame

    def _on_room_identification(self, msg: RoomIdentification) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg.annotated_frame, desired_encoding='bgr8')
        except Exception as exc:
            self._log.error(f'CameraViewerNode: room annotated frame conversion error: {exc}')
            return

        label = f"Room: {msg.room_name}  conf={msg.confidence:.2f}"
        cv2.putText(
            frame, label,
            (8, 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 200, 255), 2, cv2.LINE_AA,
        )
        self._frames[_RI_WINDOW] = frame

    # ------------------------------------------------------------------
    # Display (called from the main loop)

    def show_frames(self) -> None:
        """Render the latest frame for every known window."""
        for title, frame in self._frames.items():
            if frame is not None:
                cv2.imshow(title, frame)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraViewerNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.show_frames()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or Esc
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
