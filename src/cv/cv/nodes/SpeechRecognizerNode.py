import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils.Configurator import Configurator
from cv.services.SpeechRecognizer import SpeechRecognizer


class SpeechRecognizerNode(Node):

    def __init__(self):
        super().__init__("speech_recognizer_node")

        self.config = Configurator("cv").fetchData(
            Configurator.SPEECH_RECOGNIZER
        )

        device_index, device_channels, device_rate = SpeechRecognizer.select_mic_device(
            preferred_rate=self.config.get("sample_rate", 16000),
            config=self.config,
        )

        self.speech_recognizer = SpeechRecognizer(
            self.config,
            device_index=device_index,
            device_channels=device_channels,
            device_rate=device_rate,
        )

        self.robot_pub = self.create_publisher(String, "/commanded_robot", 10)
        self.legacy_robot_pub = self.create_publisher(String, "/robot", 10)
        self.room_pub = self.create_publisher(String, "/commanded_room", 10)
        self.object_pub = self.create_publisher(String, "/commanded_object", 10)
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
                for room, obj, action, robot in self.speech_recognizer.recognizeSpeech():
                    if any([room, obj, action, robot]):
                        self._publish_results(room, obj, action, robot)
            except Exception as e:
                self.get_logger().error(f"SpeechRecognizer error: {e}")
                continue

    # =========================================================
    # PUBLISH (executor thread)
    # =========================================================

    def _publish_results(self, room: str, obj: str, action: str, robot: str):

        if robot:
            msg = String()
            msg.data = robot.upper()
            self.robot_pub.publish(msg)
            self.legacy_robot_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Robot: {msg.data}")

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