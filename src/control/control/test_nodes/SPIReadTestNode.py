from rclpy.node import Node
from utils.Dispatcher import Dispatcher
import struct
import rclpy

class SPIReadTestNode(Node):
    def __init__(self):
        super().__init__('spi_read_test_node')
        self.commHandler = Dispatcher().get_communication_handler("ESP")
        self.timer = self.create_timer(0.1, self.testRead)
        self._logger = self.get_logger()

    def testRead(self):
        fmt = '<ffff'
        data = self.commHandler.receiveData()
        unpacked_data = struct.unpack(fmt, bytes(data))
        self._logger.info(f"Recieved Bytes: {unpacked_data}")

def main(args=None):
    rclpy.init(args=args)
    spi_read_test_node = SPIReadTestNode()
    try:
        rclpy.spin(spi_read_test_node)
    except Exception as e:
        spi_read_test_node._logger.info(f"SPIReadTestNode stopped: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        spi_read_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()