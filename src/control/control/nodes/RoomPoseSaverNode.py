import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from control.services.RoomPoseSaver import RoomPoseSaver
from control.services.JoystickPostProcessors import JoystickPostProcessors

class RoomPoseSaverNode(Node):
    def __init__(self):
        super().__init__('room_pose_saver_node')
        self._logger = self.get_logger()
        self.joystick = JoystickPostProcessors()
        self.pose_saver = RoomPoseSaver()

        #Poses 
        self.x_pose = None
        self.y_pose = None
        self.x_orientation = None
        self.y_orientation = None
        self.z_orientation = None
        self.w_orientation = None
        self.room_name = None

        self.create_subscription(Joy, 'joy', self._joyCallback, 10)
        self.create_subscription(Odometry, 'odom', self._odomCallback, 10)
        self.create_subscription(String, 'room_name', self._roomCallback, 10)
        self._timer = self.create_timer(0.5, self._savePose)

    def _joyCallback(self, msg: Joy):
        try:
            self.joystick.updateData(msg.buttons, msg.axes)
        except ValueError as e:
            self._logger.error(str(e))

    def _odomCallback(self, msg: Odometry):
        self.x_pose = msg.pose.pose.position.x
        self.y_pose = msg.pose.pose.position.y
        self.x_orientation = msg.pose.pose.orientation.x
        self.y_orientation = msg.pose.pose.orientation.y
        self.z_orientation = msg.pose.pose.orientation.z
        self.w_orientation = msg.pose.pose.orientation.w

    def _roomCallback(self, msg: String):
        if not msg.data == "UNKNOWN":
            self.room_name = msg.data

    def _savePose(self):
        if not None in (self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation, self.room_name):
            if self.joystick.isPressed('KITCHEN_SAVE'):
                self.pose_saver.updatePose('KITCHEN', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)

            if self.joystick.isPressed('LIVING_ROOM_SAVE'):
                self.pose_saver.updatePose('LIVING_ROOM', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)

            if self.joystick.isPressed('BEDROOM_SAVE'):
                self.pose_saver.updatePose('BEDROOM', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)

            if self.joystick.isPressed('BATHROOM_SAVE'):
                self.pose_saver.updatePose('BATHROOM', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)
                
            if self.joystick.isPressed('KIDS_ROOM_SAVE'):
                self.pose_saver.updatePose('KIDS_ROOM', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)

            if self.joystick.isPressed('HALLWAY_SAVE'):
                self.pose_saver.updatePose('HALLWAY', self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)
            
            if self.joystick.isPressed('GENERAL_ROOM_SAVE'):
                self.pose_saver.updatePose(self.room_name, self.x_pose, self.y_pose, self.x_orientation, self.y_orientation, self.z_orientation, self.w_orientation)
    
def main(args=None):
    rclpy.init(args=args)
    room_pose_saver_node = RoomPoseSaverNode()
    rclpy.spin(room_pose_saver_node)
    room_pose_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()