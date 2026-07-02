import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from utils.UtilityMethods import UtilityMethods
from cv.services.RoomIdentifier import RoomIdentifier

class RoomIdentifierNode(Node):
    def __init__(self):
        super().__init__('room_identifier_node')
        self.bridge = CvBridge()
        model_path = UtilityMethods.getPackageModels('cv')
        self.room_identifier = RoomIdentifier(f"{model_path}/yolov8n.pt")
        self._logger = self.get_logger()
        
        self.room_publisher = self.create_publisher(String, '/room_name', 10)
        self.room_name_msg = String()

        self.create_subscription(Image, '/image_raw', self._imageCallback, 10)
    
    def _imageCallback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        room_name = self.room_identifier.identifyRoom(frame)
        
        if room_name is not None:
            self.room_name_msg.data = room_name.upper()
            self.room_publisher.publish(self.room_name_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RoomIdentifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()