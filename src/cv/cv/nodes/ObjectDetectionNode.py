import rclpy 
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv.services.ObjectDetector import ObjectDetector

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()
        self.object_detector = ObjectDetector()
        
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self._imageCallback, 10)
        self.object_publisher = self.create_publisher(String, '/detected_object', 10)
        self.object_name_msg = String()
        self.detected_object = None
    
    def _imageCallback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detected_object = self.object_detector.detectObjects(frame)
        
        if not self.detected_object == None:
            self.object_name_msg.data = self.detected_object.upper()
            self.object_publisher.publish(self.object_name_msg)
            self.get_logger().info(f"Published Detected Object: {self.detected_object.upper()}")

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()