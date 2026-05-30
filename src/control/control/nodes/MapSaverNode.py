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

        self.map_name = "/home/mansour/Desktop/Wheelchair/maps/turtlebot"
        self.create_subscription(Joy, 'joy', self._joyCallback, 10)
        self._timer = self.create_timer(0.5, self._saveMap)

    def _joyCallback(self, msg: Joy):
        try:
            self.joystick.updateData(msg.buttons, msg.axes)
        except ValueError as e:
            self._logger.error(str(e))

    def _saveMap(self):
        if self.joystick.isPressed('MAP_SAVE'):
            if not self.serialize_client.service_is_ready() or not self.map_client.service_is_ready():
                self._logger.error('SLAM Toolbox services not available, cannot save map.')
                return
            self.__saveSerializedMap()
            self.__saveMapYaml()
            self.get_logger().info('Map + posegraph save requested.')

    def __saveSerializedMap(self):
        serialize_map_req = SerializePoseGraph.Request()
        serialize_map_req.filename = self.map_name
        future = self.serialize_client.call_async(serialize_map_req)
        future.add_done_callback(self.__serializeMapResultCallback)

    def __serializeMapResultCallback(self, future):
        try:
            future.result()
            self._logger.info('Pose graph serialized successfully.')
        except Exception as e:
            self._logger.error(f'Failed to serialize pose graph: {e}')

    def __saveMapYaml(self):
        save_map_req = SaveMap.Request()
        save_map_req.name = String()
        save_map_req.name.data = self.map_name
        future = self.map_client.call_async(save_map_req)
        future.add_done_callback(self.__saveMapResultCallback)

    def __saveMapResultCallback(self, future):
        try:
            future.result()
            self._logger.info('Map YAML saved successfully.')
        except Exception as e:
            self._logger.error(f'Failed to save map YAML: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
if __name__ == '__main__':
    main()