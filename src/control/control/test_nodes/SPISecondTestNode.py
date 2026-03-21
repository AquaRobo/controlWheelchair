from rclpy.node import Node
from utils.Dispatcher import Dispatcher
import rclpy

class SPISecondTestNode(Node):
    def __init__(self):
        super().__init__('spi_second_test_node')
        self.commHandler = Dispatcher().get_communication_handler("ESP")
        self.timer = self.create_timer(0.1, self.testSend)
        self._logger = self.get_logger()

    def testSend(self):
        data = ["s", 50.5, -50.5, 25.0]
        self._logger.info(f"Data: {data}")
        self.commHandler.sendData(data)

def main(args=None):
    rclpy.init(args=args)
    spi_second_test_node = SPISecondTestNode()
    try:
        rclpy.spin(spi_second_test_node)
    except Exception as e:
        spi_second_test_node._logger.info(f"SPISecondTestNode stopped: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        spi_second_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()