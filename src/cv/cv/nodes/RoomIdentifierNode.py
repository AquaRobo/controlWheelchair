import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.Configurator import Configurator
from cv.services.RoomIdentifier import RoomIdentifier
from my_robot_interfaces.msg import RoomIdentification
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn

_QOS_DEPTH = 10

class RoomIdentifierNode(LifecycleNode):

    def __init__(self):
        super().__init__('room_identifier_node')
        self._bridge = CvBridge()
        self._room_identifier: RoomIdentifier | None = None
        self._publisher = None
        self._active = False
        self._logger = self.get_logger()

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Configuring room identifier node...')

        # Load room identification config
        try:
            config: dict = Configurator('cv').fetchData(Configurator.ROOM_IDENTIFIER)
        except Exception as e:
            self._logger.error(f'Failed to load room_identification.yaml: {e}')
            return TransitionCallbackReturn.ERROR

        if not config:
            self._logger.error('room_identification.yaml is empty or missing.')
            return TransitionCallbackReturn.ERROR

        # Load camera config to resolve topic name
        try:
            cameras_cfg: dict = Configurator('cv').fetchData(Configurator.CAMERAS)
        except Exception as e:
            self._logger.error(f'Failed to load cameras.yaml: {e}')
            return TransitionCallbackReturn.ERROR

        camera_name: str = config.get('camera_name', '')
        if camera_name not in cameras_cfg:
            self._logger.error(
                f"Camera '{camera_name}' not found in cameras.yaml. "
                f"Available: {list(cameras_cfg.keys())}"
            )
            return TransitionCallbackReturn.ERROR

        # Initialize the room identifier service
        try:
            self._room_identifier = RoomIdentifier(config)
        except Exception as e:
            self._logger.error(f'Failed to initialize RoomIdentifier: {e}')
            return TransitionCallbackReturn.ERROR

        self._publisher = self.create_publisher(
            RoomIdentification, '/room_identification', _QOS_DEPTH
        )

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=_QOS_DEPTH,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Use the left channel for stereo cameras, plain channel for mono
        cam_cfg = cameras_cfg[camera_name]
        if cam_cfg.get('is_stereo', False):
            topic = f'/{camera_name}/left/uncalibrated'
        else:
            topic = f'/{camera_name}/uncalibrated'

        self.create_subscription(Image, topic, self._image_callback, image_qos)

        self._logger.info(
            f"Room identifier configured — camera: '{camera_name}', "
            f"topic: '{topic}', model: {config.get('model_path')}"
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Activating room identifier node...')
        self._active = True
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Deactivating room identifier node...')
        self._active = False
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Cleaning up room identifier node...')
        self._active = False
        self._room_identifier = None
        self._publisher = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Shutting down room identifier node...')
        self._active = False
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Image callback
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        if not self._active:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self._logger.error(f'Image conversion failed: {e}')
            return

        try:
            result = self._room_identifier.identifyRoom(frame)
        except Exception as e:
            self._logger.error(f'Room identification failed: {e}', throttle_duration_sec=5.0)
            return

        stamp = self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id

        out = RoomIdentification()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.room_name = result.room_name.upper()
        out.confidence = result.confidence

        try:
            ann = self._bridge.cv2_to_imgmsg(result.annotated_frame, encoding='bgr8')
            ann.header.stamp = stamp
            ann.header.frame_id = frame_id
            out.annotated_frame = ann
        except Exception as e:
            self._logger.error(f'Annotated frame conversion failed: {e}')

        self._publisher.publish(out)
        self._logger.info(
            f"Room: '{result.room_name}'  conf={result.confidence:.2f}",
            throttle_duration_sec=1.0,
        )

def main(args=None):
    rclpy.init(args=args)
    node = RoomIdentifierNode()
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
