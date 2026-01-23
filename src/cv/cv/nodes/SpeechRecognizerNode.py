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

        model_dir = UtilityMethods.getPackageModels("cv")
        model_path = f"{model_dir}/speech_recognizer.pt"

        self.speech_recognizer = SpeechRecognizer(
            self.config,
            model_path
        )

        self.room_pub = self.create_publisher(
            String,
            "/commanded_room",
            10
        )

        self.object_pub = self.create_publisher(
            String,
            "/commanded_object",
            10
        )

        self.timer = self.create_timer(0.1, self._run)

        self.get_logger().info("SpeechRecognizerNode started")


    def _run(self):
        room, obj = self.speech_recognizer.recognizeSpeech()

        if room:
            msg = String()
            msg.data = room.upper()
            self.room_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Room: {msg.data}")

        if obj:
            msg = String()
            msg.data = obj.upper()
            self.object_pub.publish(msg)
            self.get_logger().info(f"Published Commanded Object: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
