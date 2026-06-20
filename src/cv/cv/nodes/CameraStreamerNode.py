import rclpy
from cv_bridge import CvBridge
from rclpy.lifecycle import State
from sensor_msgs.msg import Image
from utils.Configurator import Configurator
from cv.services.CameraStreamer import CameraStreamer
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

_QOS_DEPTH = 10

class CameraStreamerNode(LifecycleNode):

    def __init__(self):
        super().__init__('camera_streamer_node')
        self._bridge = CvBridge()
        self._streamers: dict[str, CameraStreamer] = {}
        self._cam_publishers: dict[str, dict] = {}
        self._cam_timers: list = []
        self._logger = self.get_logger()

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Configuring camera streamer node...')
        try:
            cameras_config: dict = Configurator('cv').fetchData(Configurator.CAMERAS)
        except Exception as e:
            self._logger.error(f'Failed to load cameras config: {e}')
            return TransitionCallbackReturn.ERROR

        if not cameras_config:
            self._logger.error('cameras.yaml is empty or could not be loaded.')
            return TransitionCallbackReturn.ERROR

        for camera_name, config in cameras_config.items():
            streamer = CameraStreamer(camera_name, config)
            self._streamers[camera_name] = streamer
            self._cam_publishers[camera_name] = self._create_publishers(camera_name, streamer.is_stereo)
            self._logger.info(
                f"Registered camera '{camera_name}' "
                f"({'stereo' if streamer.is_stereo else 'mono'})"
            )

        self._logger.info('Camera streamer node configured successfully.')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Activating camera streamer node...')
        active_cameras = []

        for name, streamer in self._streamers.items():
            self._logger.info(f"Opening camera '{name}'...")
            if streamer.open():
                self._logger.info(f"Camera '{name}' opened successfully.")
                active_cameras.append(name)
            else:
                self._logger.error(
                    f"Camera '{name}' could not be opened after 10 s — skipping."
                )

        if not active_cameras:
            self._logger.error('All cameras failed to open. Cannot activate.')
            return TransitionCallbackReturn.FAILURE

        for name in active_cameras:
            fps = self._streamers[name]._config.get('fps', 30)
            period = 1.0 / float(fps)
            timer = self.create_timer(
                period,
                self._make_timer_callback(self._streamers[name], self._cam_publishers[name])
            )
            self._cam_timers.append(timer)

        self._logger.info(
            f'Camera streamer node active with cameras: {active_cameras}'
        )
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Deactivating camera streamer node...')
        for timer in self._cam_timers:
            timer.cancel()
        self._cam_timers.clear()

        for streamer in self._streamers.values():
            streamer.release()

        self._logger.info('Camera streamer node deactivated.')
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Cleaning up camera streamer node...')
        self._cam_publishers.clear()
        self._streamers.clear()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._logger.info('Shutting down camera streamer node...')
        for streamer in self._streamers.values():
            streamer.release()
        self._streamers.clear()
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Publisher creation
    # ------------------------------------------------------------------

    def _create_publishers(self, name: str, is_stereo: bool) -> dict:
        pubs = {}
        image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=_QOS_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        if is_stereo:
            pubs['left_raw']  = self.create_publisher(Image, f'/{name}/left/uncalibrated', image_qos)
            pubs['left_cal']  = self.create_publisher(Image, f'/{name}/left/calibrated', image_qos)
            pubs['right_raw'] = self.create_publisher(Image, f'/{name}/right/uncalibrated', image_qos)
            pubs['right_cal'] = self.create_publisher(Image, f'/{name}/right/calibrated', image_qos)
        else:
            pubs['raw'] = self.create_publisher(Image, f'/{name}/uncalibrated', image_qos)
            pubs['cal'] = self.create_publisher(Image, f'/{name}/calibrated', image_qos)
        return pubs

    # ------------------------------------------------------------------
    # Timer callback factory
    # ------------------------------------------------------------------

    def _make_timer_callback(self, streamer: CameraStreamer, pubs: dict):
        def _callback():
            frame_data = streamer.read()
            if frame_data is None:
                self._logger.warn(
                    f"Camera '{streamer.name}' returned no frame.", throttle_duration_sec=5.0
                )
                return

            if streamer.is_stereo:
                self._publish_stereo(frame_data, pubs, streamer.name)
            else:
                self._publish_mono(frame_data, pubs, streamer.name)

        return _callback

    def _publish_mono(self, frame_data, pubs: dict, camera_name: str) -> None:
        try:
            stamp = self.get_clock().now().to_msg()
            raw_msg = self._bridge.cv2_to_imgmsg(frame_data.raw, encoding='bgr8')
            cal_msg = self._bridge.cv2_to_imgmsg(frame_data.calibrated, encoding='bgr8')
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = camera_name
            cal_msg.header.stamp = stamp
            cal_msg.header.frame_id = camera_name
            pubs['raw'].publish(raw_msg)
            pubs['cal'].publish(cal_msg)
        except Exception as e:
            self._logger.error(f'Failed to publish mono frame: {e}')

    def _publish_stereo(self, frame_data, pubs: dict, camera_name: str) -> None:
        try:
            stamp = self.get_clock().now().to_msg()
            for key, img in (
                ('left_raw',  frame_data.left_raw),
                ('left_cal',  frame_data.left_calibrated),
                ('right_raw', frame_data.right_raw),
                ('right_cal', frame_data.right_calibrated),
            ):
                msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
                msg.header.stamp = stamp
                msg.header.frame_id = camera_name
                pubs[key].publish(msg)
        except Exception as e:
            self._logger.error(f'Failed to publish stereo frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamerNode()
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
