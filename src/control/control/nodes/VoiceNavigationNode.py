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

import math
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

        # ── Runtime safety/debug configuration ─────────────────────────────
        self._debug_logging = bool(voice_nav_cfg.get("debug_logging", False))
        self._max_encoder_dt_ns = int(voice_nav_cfg.get("max_encoder_dt_s", 0.35) * 1e9)
        self._sensor_timeout_ns = int(voice_nav_cfg.get("sensor_timeout_s", 0.5) * 1e9)
        self._linear_time_goal_ns = int(voice_nav_cfg.get("linear_time_goal_s", 5.0) * 1e9)
        self._max_primitive_duration_ns = int(
            voice_nav_cfg.get("max_primitive_duration_s", 12.0) * 1e9
        )
        self._progress_log_interval_ns = int(
            voice_nav_cfg.get("progress_log_interval_s", 0.2) * 1e9
        )

        # ── ROS I/O ───────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel_speech", 10)

        self.create_subscription(String, "/commanded_action", self._action_callback, 10)
        self.create_subscription(Encoders, "/encoders", self._encoder_callback, 10)
        self.create_subscription(Imu, "/imu", self._imu_callback, 10)

        # ── Publish timer ─────────────────────────────────────────────────
        self._publish_timer = self.create_timer(_PUBLISH_PERIOD, self._publish_cmd)

        # ── Encoder timing ────────────────────────────────────────────────
        now = self.get_clock().now()
        self._prev_encoder_time = now
        self._primitive_start_time = None
        self._last_encoder_update_time = None
        self._last_imu_update_time = None
        self._last_progress_log_time = now

        linear_accum, linear_goal = self._navigator.get_linear_progress()
        angular_accum, angular_goal = self._navigator.get_angular_progress()
        self._logger.info(
            "VoiceNavigationNode started. "
            f"linear_goal={linear_goal:.3f}m angular_goal={math.degrees(angular_goal):.1f}deg "
            f"linear_time_goal={self._linear_time_goal_ns/1e9:.2f}s "
            f"linear_speed={voice_nav_cfg.get('linear_speed', 0.2):.3f}m/s "
            f"angular_speed={voice_nav_cfg.get('angular_speed', 0.3):.3f}rad/s "
            f"debug_logging={self._debug_logging}"
        )

    # ── Subscription callbacks ────────────────────────────────────────────────

    def _action_callback(self, msg: String) -> None:
        """Receive a voice action, preempt any running primitive, start the new one."""
        action = msg.data.upper().strip()

        if self._navigator.is_active():
            old_action = self._navigator.get_active_action()
            self._logger.info(
                f"Preempting active primitive {old_action} -> {action}"
            )
            # Publish an immediate stop before switching
            self._publish_stop()
            self._navigator.reset()

        started = self._navigator.start_primitive(action)
        if started:
            now = self.get_clock().now()
            self._primitive_start_time = now
            self._prev_encoder_time = now
            self._last_encoder_update_time = None
            self._last_imu_update_time = None
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

        if dt_ns > self._max_encoder_dt_ns:
            self._logger.warn(
                f"Dropping encoder sample with oversized dt={dt_ns/1e9:.3f}s "
                f"(limit={self._max_encoder_dt_ns/1e9:.3f}s)"
            )
            self._last_encoder_update_time = now
            return

        counted = self._navigator.update_linear(msg.left_speed, msg.right_speed, dt_ns)
        self._last_encoder_update_time = now
        if self._debug_logging:
            self._maybe_log_linear_progress(now, counted, dt_ns)
        self._check_completion()

    def _imu_callback(self, msg: Imu) -> None:
        """Feed IMU quaternion into the navigator for yaw accumulation."""
        if not self._navigator.is_active() or not self._navigator.is_angular():
            return

        q = msg.orientation
        counted = self._navigator.update_angular(q.x, q.y, q.z, q.w)
        now = self.get_clock().now()
        self._last_imu_update_time = now
        if self._debug_logging:
            self._maybe_log_angular_progress(now, counted)
        self._check_completion()

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_cmd(self) -> None:
        """Stream /cmd_vel_speech at fixed rate while a primitive is active."""
        if not self._navigator.is_active():
            return

        now = self.get_clock().now()
        if self._is_primitive_duration_exceeded(now):
            self._safety_stop("primitive max duration exceeded")
            return
        if self._is_sensor_timeout(now):
            self._safety_stop("sensor watchdog timeout")
            return

        # Keep completion checks in timer as well to avoid dependency on callback cadence.
        self._check_completion()
        if not self._navigator.is_active():
            return

        lin_x, ang_z = self._navigator.get_command()
        self._send_twist(lin_x, ang_z)

        if self._debug_logging:
            action = self._navigator.get_active_action()
            self._logger.debug(
                f"Streaming cmd for {action}: linear_x={lin_x:.3f} angular_z={ang_z:.3f}"
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _check_completion(self) -> None:
        """If the navigator reports completion, stop and clear state."""
        if not self._navigator.is_active():
            return

        # Encoders are currently unavailable: complete linear primitives by elapsed time.
        if self._navigator.is_linear() and self._primitive_start_time is not None:
            now = self.get_clock().now()
            elapsed_ns = (now - self._primitive_start_time).nanoseconds
            if elapsed_ns >= self._linear_time_goal_ns:
                action = self._navigator.get_active_action()
                self._logger.info(
                    f"Primitive complete: {action} (time-based, elapsed={elapsed_ns/1e9:.2f}s)"
                )
                self._navigator.reset()
                self._primitive_start_time = None
                self._publish_stop()
            return

        if self._navigator.is_complete():
            action = self._navigator.get_active_action()
            linear_accum, linear_goal = self._navigator.get_linear_progress()
            angular_accum, angular_goal = self._navigator.get_angular_progress()
            self._logger.info(f"Primitive complete: {action}")
            if self._debug_logging:
                self._logger.info(
                    "Completion progress "
                    f"linear={linear_accum:.3f}/{linear_goal:.3f}m "
                    f"angular={math.degrees(angular_accum):.1f}/{math.degrees(angular_goal):.1f}deg"
                )
            self._navigator.reset()
            self._primitive_start_time = None
            self._publish_stop()

    def _publish_stop(self) -> None:
        """Publish a single zero Twist to halt the chair."""
        self._logger.info("Publishing stop command (0.0, 0.0)")
        self._send_twist(0.0, 0.0)

    def _safety_stop(self, reason: str) -> None:
        """Force-stop active primitive due to timeout/invalid sensor cadence."""
        action = self._navigator.get_active_action()
        self._logger.warn(f"Safety stop for {action}: {reason}")
        self._navigator.reset()
        self._primitive_start_time = None
        self._publish_stop()

    def _is_sensor_timeout(self, now) -> bool:
        """True when required sensor updates have stalled for current primitive."""
        if not self._navigator.is_active():
            return False

        # Linear primitives are time-based for now, so don't watchdog on encoder cadence.
        if self._navigator.is_linear():
            return False

        ref = self._last_imu_update_time or self._primitive_start_time

        if ref is None:
            return False
        return (now - ref).nanoseconds > self._sensor_timeout_ns

    def _is_primitive_duration_exceeded(self, now) -> bool:
        """True when primitive runtime exceeded hard maximum duration."""
        if not self._navigator.is_active() or self._primitive_start_time is None:
            return False
        return (now - self._primitive_start_time).nanoseconds > self._max_primitive_duration_ns

    def _maybe_log_linear_progress(self, now, counted_m: float, dt_ns: int) -> None:
        """Throttled debug log for linear primitive progress."""
        if (now - self._last_progress_log_time).nanoseconds < self._progress_log_interval_ns:
            return
        self._last_progress_log_time = now
        accum, goal = self._navigator.get_linear_progress()
        action = self._navigator.get_active_action()
        self._logger.info(
            f"Linear progress {action}: +{counted_m:.4f}m in dt={dt_ns/1e9:.3f}s, total={accum:.4f}/{goal:.4f}m"
        )

    def _maybe_log_angular_progress(self, now, counted_rad: float) -> None:
        """Throttled debug log for angular primitive progress."""
        if (now - self._last_progress_log_time).nanoseconds < self._progress_log_interval_ns:
            return
        self._last_progress_log_time = now
        accum, goal = self._navigator.get_angular_progress()
        action = self._navigator.get_active_action()
        self._logger.info(
            "Angular progress "
            f"{action}: +{math.degrees(counted_rad):.2f}deg, "
            f"total={math.degrees(accum):.2f}/{math.degrees(goal):.2f}deg"
        )

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