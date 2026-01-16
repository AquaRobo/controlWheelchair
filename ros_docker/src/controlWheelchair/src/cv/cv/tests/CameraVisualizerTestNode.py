import rclpy
from rclpy.node import Node
import cv2 as cv 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraVisualizerTestNode(Node):
    def __init__(self):
        super().__init__('camera_visualizer_test_node')
        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        self.get_logger().info("Camera Visualizer Node has been started.")
        self.latest_frame = None
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_frame = frame

        # Visualize
        cv.imshow('Camera Feed', frame)
        cv.waitKey(1)
    

def main(args=None):
    rclpy.init(args=args)
    node = CameraVisualizerTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv.destroyAllWindows()
if __name__ == '__main__':
    main()