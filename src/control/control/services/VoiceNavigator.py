"""
VoiceNavigator.py — Pure calculation service for voice-commanded motion primitives.

Responsibilities
----------------
* Map action strings to motion goals (linear 1 m, angular ±30°).
* Accumulate signed linear displacement from encoder RPM readings.
* Accumulate yaw displacement from IMU quaternions.
* Decide when a primitive is complete.
* Generate Twist-ready (linear_x, angular_z) command values.
"""

import math


class VoiceNavigator:
    """
    Stateful calculation service for one active motion primitive at a time.

    Lifecycle per primitive
    -----------------------
    1. ``start_primitive(action)``  — sets goal, direction, resets accumulators.
    2. ``update_linear(...)``       — called on every encoder message (FORWARD/BACKWARD).
    3. ``update_angular(...)``      — called on every IMU message (LEFT/RIGHT).
    4. ``is_complete()``            — True when accumulated progress ≥ goal.
    5. ``get_command()``            — returns (linear_x, angular_z) for /cmd_vel_speech.
    6. ``reset()``                  — clears state (called on preemption or completion).
    """

    def __init__(self, robot_cfg: dict, voice_nav_cfg: dict) -> None:
        """
        Parameters
        ----------
        robot_cfg:
            Wheelchair configuration dictionary.
        voice_nav_cfg:
            Voice navigation configuration dictionary.
        """
        self._wheel_radius     = robot_cfg["rear_wheels_radius"]
        self._wheel_separation = robot_cfg["rear_wheels_separation"]
        self._linear_goal       = voice_nav_cfg.get("linear_goal", 1.0)
        self._angular_goal      = math.radians(voice_nav_cfg.get("angular_goal_deg", 30.0))
        self._linear_speed      = voice_nav_cfg.get("linear_speed", 0.2)
        self._angular_speed     = voice_nav_cfg.get("angular_speed", 0.3)
        self._rpm_noise_floor   = voice_nav_cfg.get("rpm_noise_floor", 0.5)
        self._rpm_to_rad_s      = math.tau / 60.0
        self._supported_actions   = set(a.upper() for a in voice_nav_cfg.get("supported_actions", []))

        # Active primitive state
        self._action: str | None = None   # "FORWARD" | "BACKWARD" | "LEFT" | "RIGHT"
        self._linear_sign: float = 0.0    # +1 or -1
        self._angular_sign: float = 0.0   # +1 or -1

        # Accumulators (unsigned progress toward goal)
        self._linear_accum:  float = 0.0  # metres accumulated
        self._angular_accum: float = 0.0  # radians accumulated

        # IMU tracking
        self._prev_yaw: float | None = None   # last known yaw [rad]

    # ── Public API ────────────────────────────────────────────────────────────

    def start_primitive(self, action: str) -> bool:
        """
        Initialise a new motion primitive.

        Returns True if the action is supported, False otherwise.
        The caller should call reset() before start_primitive() when preempting.
        """
        action = action.upper().strip()
        if action not in self._supported_actions:
            return False

        self.reset()
        self._action = action

        if action == "FORWARD":
            self._linear_sign = +1.0
        elif action == "BACKWARD":
            self._linear_sign = -1.0
        elif action == "LEFT":
            self._angular_sign = +1.0   # CCW positive (REP-103)
        elif action == "RIGHT":
            self._angular_sign = -1.0

        return True

    def reset(self) -> None:
        """Clear all primitive state.  Call before starting a new primitive."""
        self._action        = None
        self._linear_sign   = 0.0
        self._angular_sign  = 0.0
        self._linear_accum  = 0.0
        self._angular_accum = 0.0
        self._prev_yaw      = None

    def is_active(self) -> bool:
        """True while a primitive is running and not yet complete."""
        return self._action is not None

    def is_linear(self) -> bool:
        return self._action in ("FORWARD", "BACKWARD")

    def is_angular(self) -> bool:
        return self._action in ("LEFT", "RIGHT")

    # ── Encoder update (linear primitives) ───────────────────────────────────

    def update_linear(self, left_rpm: float, right_rpm: float, dt_ns: int) -> None:
        """
        Accumulate linear displacement from a new encoder reading.

        Parameters
        ----------
        left_rpm / right_rpm:
            Raw RPM values from the Encoders message.
        dt_ns:
            Time delta since the last encoder message, in nanoseconds.
        """
        if not self.is_active() or not self.is_linear():
            return
        if dt_ns <= 0:
            return

        dt_s = dt_ns / 1e9

        # Dead-band identical to OdomHardwareNode
        l_rpm = left_rpm  if abs(left_rpm)  > self._rpm_noise_floor else 0.0
        r_rpm = right_rpm if abs(right_rpm) > self._rpm_noise_floor else 0.0

        omega_l = l_rpm * self._rpm_to_rad_s   # wheel angular velocity [rad/s]
        omega_r = r_rpm * self._rpm_to_rad_s

        dp_left  = omega_l * dt_s   # wheel angular displacement [rad]
        dp_right = omega_r * dt_s

        d_s = self._wheel_radius * (dp_right + dp_left) / 2.0   # metres

        # Only count displacement in the commanded direction
        if self._linear_sign * d_s >= 0.0:
            self._linear_accum += abs(d_s)

    # ── IMU update (angular primitives) ──────────────────────────────────────

    def update_angular(self, qx: float, qy: float, qz: float, qw: float) -> None:
        """
        Accumulate yaw rotation from a new IMU quaternion.

        Uses shortest-arc / wrap-safe delta so a 30° goal works correctly
        even across the ±π boundary.
        """
        if not self.is_active() or not self.is_angular():
            return

        yaw = self._quat_to_yaw(qx, qy, qz, qw)

        if self._prev_yaw is None:
            self._prev_yaw = yaw
            return

        delta = self._wrap_angle(yaw - self._prev_yaw)
        self._prev_yaw = yaw

        # Only count rotation in the commanded direction
        if self._angular_sign * delta >= 0.0:
            self._angular_accum += abs(delta)

    # ── Completion check ─────────────────────────────────────────────────────

    def is_complete(self) -> bool:
        """True when the accumulated progress meets or exceeds the goal."""
        if not self.is_active():
            return False
        if self.is_linear():
            return self._linear_accum >= self._linear_goal
        if self.is_angular():
            return self._angular_accum >= self._angular_goal
        return False

    # ── Command generation ───────────────────────────────────────────────────

    def get_command(self) -> tuple[float, float]:
        """
        Return ``(linear_x, angular_z)`` values for a Twist message.

        Returns ``(0.0, 0.0)`` when no primitive is active.
        """
        if not self.is_active():
            return 0.0, 0.0
        if self.is_linear():
            return self._linear_sign * self._linear_speed, 0.0
        if self.is_angular():
            return 0.0, self._angular_sign * self._angular_speed
        return 0.0, 0.0

    @staticmethod
    def get_stop_command() -> tuple[float, float]:
        """Explicit zero command — publish once on completion/preemption."""
        return 0.0, 0.0

    # ── Internal helpers ─────────────────────────────────────────────────────

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        """Extract yaw from a quaternion (ZYX convention, REP-103)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to (−π, π]."""
        return (angle + math.pi) % math.tau - math.pi