"""
VoiceNavigationNode.py — ROS 2 node for voice-commanded wheelchair motion.

Subscriptions
-------------
/commanded_action  (std_msgs/String)  — FORWARD | BACKWARD | LEFT | RIGHT
/encoders          (Encoders)         — wheel RPM, feeds linear primitive
/imu/data          (sensor_msgs/Imu)  — orientation quaternion, feeds angular primitive

Publications
------------
/cmd_vel_speech    (geometry_msgs/Twist) — streamed at 20 Hz while active,
                                           zero-stop published on completion / preemption

Node responsibilities (only)
-----------------------------
* Subscribe / publish.
* Own the ROS clock and timer.
* Preempt on new action.
* Feed sensor data into VoiceNavigator and ask it for commands / completion.

VoiceNavigator responsibilities (only)
---------------------------------------
* All maths — distance integration, yaw accumulation, command generation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from my_robot_interfaces.msg import Encoders
from utils.Configurator import Configurator
from control.services.VoiceNavigator import VoiceNavigator

_PUBLISH_HZ = 20          # Hz — /cmd_vel_speech stream rate
_PUBLISH_PERIOD = 1.0 / _PUBLISH_HZ

class VoiceNavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_navigation_node")
        self._logger = self.get_logger()

        # ── Wheel geometry from shared config (mirrors OdomHardwareNode) ──
        robot_cfg = Configurator("control").fetchData(Configurator.WHEELCHAIR_CONFIG)
        voice_nav_cfg = Configurator("control").fetchData(Configurator.VOICE_NAVIGATION)

        # ── Calculation service ────────────────────────────────────────────
        self._navigator = VoiceNavigator(robot_cfg, voice_nav_cfg)

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_speech", 10)

        self.create_subscription(String, "/commanded_action", self._action_callback, 10)
        self.create_subscription(Encoders, "/encoders", self._encoder_callback, 10)
        self.create_subscription(Imu, "/imu", self._imu_callback, 10)

        # ── Publish timer ─────────────────────────────────────────────────
        self._publish_timer = self.create_timer(_PUBLISH_PERIOD, self._publish_cmd)

        # ── Encoder timing ────────────────────────────────────────────────
        self._prev_encoder_time = self.get_clock().now()

        self._logger.info("VoiceNavigationNode started.")

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _action_callback(self, msg: String) -> None:
        """Receive a voice action, preempt any running primitive, start the new one."""
        action = msg.data.upper().strip()

        if self._navigator.is_active():
            self._logger.info(
                f"Preempting active primitive — new action: {action}"
            )
            # Publish an immediate stop before switching
            self._publish_stop()
            self._navigator.reset()

        started = self._navigator.start_primitive(action)
        if started:
            self._logger.info(f"Starting primitive: {action}")
        else:
            self._logger.warn(
                f"Unsupported action '{action}' — ignored. "
                f"Supported: FORWARD, BACKWARD, LEFT, RIGHT"
            )

    def _encoder_callback(self, msg: Encoders) -> None:
        """Feed wheel RPM into the navigator for linear distance accumulation."""
        if not self._navigator.is_active() or not self._navigator.is_linear():
            # Update timing even when idle so first delta is accurate
            self._prev_encoder_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt_ns = (now - self._prev_encoder_time).nanoseconds
        self._prev_encoder_time = now

        self._navigator.update_linear(msg.left_speed, msg.right_speed, dt_ns)
        self._check_completion()

    def _imu_callback(self, msg: Imu) -> None:
        """Feed IMU quaternion into the navigator for yaw accumulation."""
        if not self._navigator.is_active() or not self._navigator.is_angular():
            return

        q = msg.orientation
        self._navigator.update_angular(q.x, q.y, q.z, q.w)
        self._check_completion()

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_cmd(self) -> None:
        """Stream /cmd_vel_speech at fixed rate while a primitive is active."""
        if not self._navigator.is_active():
            return

        lin_x, ang_z = self._navigator.get_command()
        self._send_twist(lin_x, ang_z)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _check_completion(self) -> None:
        """If the navigator reports completion, stop and clear state."""
        if self._navigator.is_complete():
            action = self._navigator._action   # log which primitive finished
            self._logger.info(f"Primitive complete: {action}")
            self._navigator.reset()
            self._publish_stop()

    def _publish_stop(self) -> None:
        """Publish a single zero Twist to halt the chair."""
        self._send_twist(0.0, 0.0)

    def _send_twist(self, linear_x: float, angular_z: float) -> None:
        twist = Twist()
        twist.linear.x  = linear_x
        twist.angular.z = angular_z
        self._cmd_pub.publish(twist)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()