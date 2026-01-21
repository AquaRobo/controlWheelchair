import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.Configurator import Configurator
from utils.UtilityMethods import UtilityMethods
from cv.services.SpeechRecognizer import SpeechRecognizer

class SpeechRecognizerNode(Node):
    def __init__(self):
        super().__init__('speech_recognizer_node')
        self.speech_recognizer_config = Configurator('cv').fetchData(Configurator.SPEECH_RECOGNIZER)
        self._logger = self.get_logger()
        self.model_path = UtilityMethods.getPackageModels('cv')
        self.speech_recognizer = SpeechRecognizer(self.speech_recognizer_config, f"{self.model_path}/speech_recognizer.pt")

        self.room_command_publisher = self.create_publisher(String, '/commanded_room', 10)
        self.room_command_msg = String()

        self.object_command_publisher = self.create_publisher(String, '/commanded_object', 10)
        self.object_command_msg = String()

        self.speech_recognized = False
        self.create_timer(0.1, self._run)

    def _publishCommandedRoom(self):
        commanded_room = self.speech_recognizer.getRecognizedRoom()
        if commanded_room is not None:
            self.room_command_msg.data = commanded_room.upper()
            self.room_command_publisher.publish(self.room_command_msg)
            self._logger.info(f"Published Commanded Room: {commanded_room.upper()}")

    def _publishCommandedObject(self):
        commanded_object = self.speech_recognizer.getRecognizedObject()
        if commanded_object is not None:
            self.object_command_msg.data = commanded_object.upper()
            self.object_command_publisher.publish(self.object_command_msg)
            self._logger.info(f"Published Commanded Object: {commanded_object.upper()}")

    def _run(self):
        self.speech_recognized = self.speech_recognizer.recognizeSpeech()
        # self._logger.info(f"Wake Word Probability: {self.speech_recognizer.getWakeWordProbability():.2f}")
        # self._logger.info(f"Transcribed Text: {self.speech_recognizer.getTranscription()}")
        if self.speech_recognized:
            self._publishCommandedRoom()
            self._publishCommandedObject()
        else:
            self.speech_recognized = False

def main(args=None):
    rclpy.init(args=args)
    speech_recognizer_node = SpeechRecognizerNode()
    rclpy.spin(speech_recognizer_node)
    speech_recognizer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()