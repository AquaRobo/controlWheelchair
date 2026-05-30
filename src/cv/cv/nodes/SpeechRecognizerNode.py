import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils.Configurator import Configurator
from utils.UtilityMethods import UtilityMethods
from cv.services.SpeechRecognizer import SpeechRecognizer


class SpeechRecognizerNode(Node):

    def __init__(self):
        super().__init__("speech_recognizer_node")

        self.config = Configurator("cv").fetchData(
            Configurator.SPEECH_RECOGNIZER
        )

        model_dir  = UtilityMethods.getPackageModels("cv")
        model_path = f"{model_dir}/best_wakeword_model3.pt"

        # ── Mic selection (terminal prompt before stream opens) ────────────
        device_index, device_channels, device_rate = SpeechRecognizer.select_mic_device(
            sample_rate=self.config.get("sample_rate", 44100)
        )

        self.speech_recognizer = SpeechRecognizer(
            self.config, model_path,
            device_index=device_index,
            device_channels=device_channels,
            device_rate=device_rate,
        )
        self.speech_recognizer.calibrate_noise_floor(seconds=10.0)

        self.room_pub    = self.create_publisher(String, "/commanded_room",   10)
        self.object_pub  = self.create_publisher(String, "/commanded_object", 10)
        self.actions_pub = self.create_publisher(String, "/commanded_action", 10)

        self._detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True
        )
        self._detection_thread.start()

        self.get_logger().info("SpeechRecognizerNode started")

    # =========================================================
    # DETECTION LOOP (background thread)
    # =========================================================

    def _detection_loop(self):
        while rclpy.ok():
            try:
                for room, obj, action in self.speech_recognizer.recognizeSpeech():
                    if any([room, obj, action]):
                        self.executor.create_task(
                            self._publish_results(room, obj, action)
                        )
            except Exception as e:
                self.get_logger().error(f"SpeechRecognizer error: {e}")
                continue

    # =========================================================
    # PUBLISH (executor thread)
    # =========================================================

    async def _publish_results(self, room: str, obj: str, action: str):
        if room:
            msg      = String()
            msg.data = room.upper()
            self.room_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Room: {msg.data}")

        if obj:
            msg      = String()
            msg.data = obj.upper()
            self.object_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Object: {msg.data}")

        if action:
            msg      = String()
            msg.data = action.upper()
            self.actions_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Action: {msg.data}")

    # =========================================================
    # CLEANUP
    # =========================================================

    def destroy_node(self):
        self.speech_recognizer.end_stream()
        super().destroy_node()


# =========================================================
# MAIN
# =========================================================

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()