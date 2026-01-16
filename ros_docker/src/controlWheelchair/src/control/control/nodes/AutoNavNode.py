import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from utils.Configurator import Configurator

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('auto_nav_node')
        self._logger = self.get_logger()
        self.room_poses_config = Configurator('control').fetchData(Configurator.ROOM_POSES)
        self.room_name = None

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose = PoseStamped()

        self.create_subscription(String, 'commanded_room', self._roomCallback, 10)
        self.create_timer(1.0, self._navigateToRoom)

    def _roomCallback(self, msg: String):
        self.room_name = msg.data

    def _navigateToRoom(self):
        if self.room_name is not None and self.room_name in self.room_poses_config:
            if not self._client.wait_for_server(timeout_sec=1.0):
                self._logger.error('NavigateToPose action server not available!')
                return

            pose_data = self.room_poses_config[self.room_name]
            self.goal_msg.pose.header.frame_id = 'map'
            self.goal_msg.pose.pose.position.x = pose_data['position']['x']
            self.goal_msg.pose.pose.position.y = pose_data['position']['y']
            self.goal_msg.pose.pose.position.z = 0.0
            self.goal_msg.pose.pose.orientation.x = pose_data['orientation']['x']
            self.goal_msg.pose.pose.orientation.y = pose_data['orientation']['y']
            self.goal_msg.pose.pose.orientation.z = pose_data['orientation']['z']
            self.goal_msg.pose.pose.orientation.w = pose_data['orientation']['w']

            self._logger.info(f'Sending navigation goal to {self.room_name}...')
            send_goal_future = self._client.send_goal_async(self.goal_msg, self.__feedbackCallback)
            send_goal_future.add_done_callback(self.__goalResponseCallback)

            self.room_name = None

    def __feedbackCallback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def __goalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.__getResultCallback)

    def __getResultCallback(self, future):
        result = future.result().result
        self._logger.info('Navigation completed successfully.')

def main(args=None):
    rclpy.init(args=args)
    auto_nav_node = AutoNavNode()
    rclpy.spin(auto_nav_node)
    auto_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    