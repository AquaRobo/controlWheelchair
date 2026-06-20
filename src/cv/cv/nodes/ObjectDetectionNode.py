import rclpy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Configurator import Configurator
from cv.services.ObjectDetector import ObjectDetector
from my_robot_interfaces.msg import ObjectDetection, ObjectPose
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

_QOS_DEPTH = 10


class ObjectDetectionNode(LifecycleNode):

    def __init__(self):
        super().__init__('object_detection_node')
        self._bridge = CvBridge()
        self._detector: ObjectDetector | None = None
        self._publisher = None
        self._sync: message_filters.ApproximateTimeSynchronizer | None = None
        self._active = False
        self._logger = self.get_logger()

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Configuring object detection node...')

        # Load detection config
        try:
            det_cfg: dict = Configurator('cv').fetchData(Configurator.OBJECT_DETECTION)
        except Exception as e:
            self._logger.error(f'Failed to load object_detection.yaml: {e}')
            return TransitionCallbackReturn.ERROR

        if not det_cfg:
            self._logger.error('object_detection.yaml is empty or missing.')
            return TransitionCallbackReturn.ERROR

        # Load camera config for depth intrinsics
        try:
            cameras_cfg: dict = Configurator('cv').fetchData(Configurator.CAMERAS)
        except Exception as e:
            self._logger.error(f'Failed to load cameras.yaml: {e}')
            return TransitionCallbackReturn.ERROR

        camera_name: str = det_cfg.get('camera_name', '')
        if camera_name not in cameras_cfg:
            self._logger.error(
                f"Camera '{camera_name}' not found in cameras.yaml. "
                f"Available: {list(cameras_cfg.keys())}"
            )
            return TransitionCallbackReturn.ERROR

        cam_cfg = cameras_cfg[camera_name]
        if not cam_cfg.get('is_stereo', False):
            self._logger.error(
                f"Camera '{camera_name}' is not stereo — "
                "depth estimation requires a stereo pair."
            )
            return TransitionCallbackReturn.ERROR

        # Merge depth intrinsics into detection config
        merged_cfg = {
            **det_cfg,
            'baseline':            cam_cfg.get('baseline'),
            'focal_length_pixels': cam_cfg.get('focal_length_pixels'),
            'disparity_offset':    cam_cfg.get('disparity_offset'),
        }

        # Initialise detector service
        try:
            self._detector = ObjectDetector(merged_cfg)
        except Exception as e:
            self._logger.error(f'Failed to initialise ObjectDetector: {e}')
            return TransitionCallbackReturn.ERROR

        # Publisher
        self._publisher = self.create_publisher(
            ObjectDetection, '/object_detection', _QOS_DEPTH
        )

        # Synchronised stereo subscriptions
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        left_sub = message_filters.Subscriber(
            self, Image, f'/{camera_name}/left/uncalibrated',
            qos_profile=image_qos,
        )
        right_sub = message_filters.Subscriber(
            self, Image, f'/{camera_name}/right/uncalibrated',
            qos_profile=image_qos,
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [left_sub, right_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._sync_callback)

        self._logger.info(
            f"Object detection configured — camera: '{camera_name}', "
            f"model: {det_cfg.get('model_path')}, "
            f"classes: {det_cfg.get('classes')}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Activating object detection node...')
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Deactivating object detection node...')
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Cleaning up object detection node...')
        self._active = False
        self._detector = None
        self._sync = None
        self._publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Shutting down object detection node...')
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Synchronised stereo callback
    # ------------------------------------------------------------------

    def _sync_callback(self, left_msg: Image, right_msg: Image) -> None:
        if not self._active:
            return

        try:
            left  = self._bridge.imgmsg_to_cv2(left_msg,  desired_encoding='bgr8')
            right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            self._logger.error(f'CvBridge conversion failed: {e}')
            return

        try:
            detections = self._detector.detect(left, right)
        except Exception as e:
            self._logger.error(f'Detection failed: {e}', throttle_duration_sec=5.0)
            return

        if not detections:
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = left_msg.header.frame_id

        for result in detections:
            msg = ObjectDetection()
            msg.header.stamp    = stamp
            msg.header.frame_id = frame_id
            msg.object_name     = result.object_name
            msg.confidence      = result.confidence
            msg.object_pose     = ObjectPose(x=result.x, y=result.y, z=result.z)

            try:
                ann = self._bridge.cv2_to_imgmsg(result.annotated_frame, encoding='bgr8')
                ann.header.stamp    = stamp
                ann.header.frame_id = frame_id
                msg.annotated_frame = ann
            except Exception as e:
                self._logger.error(f'Annotated frame conversion failed: {e}')

            self._publisher.publish(msg)
            self._logger.info(
                f"Detected '{result.object_name}' "
                f"(conf={result.confidence:.2f}) "
                f"X={result.x:.3f} Y={result.y:.3f} Z={result.z:.3f} m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
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
