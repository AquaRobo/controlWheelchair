import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class M(Node):
    def __init__(self):
        super().__init__('m')
        self.sub = self.create_subscription(Image, '/lsm36156/left/image_raw', self.cb, 10)
    def cb(self, msg):
        print(f"Received frame. Size: {len(msg.data)}. Is all zero? {all(x == 0 for x in msg.data)}")
        exit(0)

rclpy.init()
node = M()
rclpy.spin(node)
