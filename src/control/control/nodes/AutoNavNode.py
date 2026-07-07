import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from utils.Configurator import Configurator

class AutoNavNode(Node):
    """Sends Nav2 goals to drive the wheelchair to named rooms.

    Listens for a room name, looks up its saved pose in room_poses.yaml
    (written by RoomPoseSaverNode), and sends it as a NavigateToPose action
    goal in the 'map' frame. A new room command preempts (cancels) any goal
    that is still in flight.

    Subscriptions:
        commanded_room (std_msgs/String): target room name; must match a key
            in room_poses.yaml.

    Action clients:
        navigate_to_pose (nav2_msgs/NavigateToPose): Nav2 navigation server.
    """

    def __init__(self):
        super().__init__('auto_nav_node')
        self._logger = self.get_logger()
        self.room_poses_config = Configurator('control').fetchData(Configurator.ROOM_POSES)
        self.room_name = None
        self._active_goal_handle = None

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose = PoseStamped()

        self.create_subscription(String, 'commanded_room', self._roomCallback, 10)
        self.create_timer(1.0, self._navigateToRoom)

    def _roomCallback(self, msg: String):
        self.room_name = msg.data
        # Cancel any in-flight goal so the robot heads to the new room
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    def _navigateToRoom(self):
        if self.room_name is None or self.room_name not in self.room_poses_config:
            return

        if not self._client.server_is_ready():
            self._logger.error('NavigateToPose action server not available!')
            return

        pose_data = self.room_poses_config[self.room_name]
        self.goal_msg.pose.header.frame_id = 'map'
        self.goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
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
        pass

    def __goalResponseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._logger.warn('Navigation goal was rejected by the action server.')
            return

        self._active_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.__getResultCallback)

    def __getResultCallback(self, future):
        self._active_goal_handle = None
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._logger.info('Navigation completed successfully.')
        elif status == GoalStatus.STATUS_CANCELED:
            self._logger.info('Navigation goal was cancelled.')
        else:
            self._logger.error(f'Navigation failed with status: {status}')

def main(args=None):
    rclpy.init(args=args)
    auto_nav_node = AutoNavNode()
    try:
        rclpy.spin(auto_nav_node)
    except KeyboardInterrupt:
        pass
    finally:
        auto_nav_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

    