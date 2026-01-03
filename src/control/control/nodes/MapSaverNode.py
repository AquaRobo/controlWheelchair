import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from control.services.JoystickPostProcessors import JoystickPostProcessors

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self._logger = self.get_logger()
        self.joystick = JoystickPostProcessors()

        # Clients
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.map_client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        for client, name in [(self.serialize_client, 'serialize_map'), (self.map_client, 'save_map')]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {name}...')

        self.map_name = "/home/mansour/Desktop/EWheelchair/maps/turtlebot"
        self.create_subscription(Joy, 'joy', self._joyCallback, 10)
        self._timer = self.create_timer(0.5, self._saveMap)

    def _joyCallback(self, msg: Joy):
        try:
            self.joystick.updateData(msg.buttons, msg.axes)
        except ValueError as e:
            self._logger.error(str(e))

    def _saveMap(self):
        if self.joystick.isPressed('MAP_SAVE'):
            self.__saveSerializedMap()
            self.__saveMapYaml()
            self.get_logger().info('Map + posegraph saved!')

    def __saveSerializedMap(self):
        serialize_map_req = SerializePoseGraph.Request()
        serialize_map_req.filename = self.map_name
        self.serialize_client.call_async(serialize_map_req)

    def __saveMapYaml(self):
        save_map_req = SaveMap.Request()
        save_map_req.name = String()
        save_map_req.name.data = self.map_name
        self.map_client.call_async(save_map_req)

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()