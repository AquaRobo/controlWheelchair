from control.services.MotorDriver import MotorDriver, START_FRAME
from my_robot_interfaces.msg import MotorSpeeds, Encoders
from rclpy.node import Node
import struct
import rclpy

FEEDBACK_FMT = '<HhhhhHhHH'
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FMT)

class HoverBoardNode(Node):
    def __init__(self):
        super().__init__('hoverboard_node')
        self._logger = self.get_logger()
        self.motor_driver = MotorDriver()
        self.encoders_msg = Encoders()

        self.motor_speeds_sub = self.create_subscription(
            MotorSpeeds, '/motor_speeds', self._motorSpeedsCallback, 10
        )
        self.encoders_pub = self.create_publisher(Encoders, '/encoders', 10)

    def _motorSpeedsCallback(self, msg: MotorSpeeds) -> None:
        self.motor_driver.drivePwm(msg.right_motor, msg.left_motor)
        raw = self.motor_driver.commHandler.receiveData()
        self._publishEncoders(raw)

    def _publishEncoders(self, raw) -> None:
        if raw:
            raw = bytes(raw) if not isinstance(raw, (bytes, bytearray)) else raw
        feedback = self.__parse_feedback(raw) if raw else None
        if feedback is not None:
            self.encoders_msg.right_speed = float(feedback['speedR'])
            self.encoders_msg.left_speed = float(feedback['speedL'])
        else:
            if raw:
                self._logger.warn('Feedback parse error: invalid or incomplete frame')
            self.encoders_msg.right_speed = 0.0
            self.encoders_msg.left_speed = 0.0
        self.encoders_pub.publish(self.encoders_msg)

    def __parse_feedback(self, data: bytes):
        """
        Parse an 18-byte feedback frame from the hoverboard.

        Returns a dict with keys:
          cmd1        – echoed command 1 (steer)
          cmd2        – echoed command 2 (speed)
          speedR      – right motor measured speed (rpm)
          speedL      – left motor measured speed (rpm)
          batVoltage  – battery voltage  (divide by 100 → volts)
          boardTemp   – board temperature (divide by 10 → °C)
          cmdLed      – LED state bits
        Returns None if the frame is invalid (bad start or checksum).
        """
        if len(data) < FEEDBACK_SIZE:
            return None
        fields = struct.unpack(FEEDBACK_FMT, data[:FEEDBACK_SIZE])
        start, cmd1, cmd2, speedR, speedL, batVoltage, boardTemp, cmdLed, checksum = fields
        if start != START_FRAME:
            return None
        expected = (START_FRAME ^ (cmd1 & 0xFFFF) ^ (cmd2 & 0xFFFF)
                    ^ (speedR & 0xFFFF) ^ (speedL & 0xFFFF)
                    ^ (batVoltage & 0xFFFF) ^ (boardTemp & 0xFFFF) ^ cmdLed)
        if (expected & 0xFFFF) != checksum:
            return None
        return {
            'cmd1':       cmd1,
            'cmd2':       cmd2,
            'speedR':     speedR,
            'speedL':     -speedL,
            'batVoltage': batVoltage / 100.0,
            'boardTemp':  boardTemp / 10.0,
            'cmdLed':     cmdLed,
        }


def main(args=None):
    rclpy.init(args=args)
    hoverboard_node = HoverBoardNode()
    try:
        rclpy.spin(hoverboard_node)
    except KeyboardInterrupt:
        pass
    finally:
        hoverboard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
