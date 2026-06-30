import rclpy
import math
import threading
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from utils.Dispatcher import Dispatcher

class Steppers(Node):
    def __init__(self):
        super().__init__('Steppers')

        # 🔹 Joint names (must match /joint_states)
        self.joint_names = [
            'Joint_1',
            'Joint_2',
            'Joint_3',
            'Joint_4',
            'Joint_5',
            'Joint_6'
        ]

        # 🔹 Storage
        self.current_joints = [0.0] * len(self.joint_names)
        self.joint_states_received = False
        self.pending_steps = None      # Latest converted steps waiting for final send
        self.stable_cycles = 0         # Count how many timer cycles the packet was unchanged
        self.stability_threshold = 3   # Send only after this many unchanged cycles
        self.last_sent_steps = None    # Track last sent packet to avoid duplicates

        # 🔹 Motor configuration
        self.steps_per_rev = 200       # 1.8° stepper
        self.microstepping = 16        # driver setting
        self.gear_ratio = 1            # adjust if gearbox exists
        self.total_steps_per_rev = (   # fixed: was [self.total](http://...)
            self.steps_per_rev *
            self.microstepping *
            self.gear_ratio
        )

        # 🔹 Calibration
        # self.offsets = [0.0] * len(self.joint_names)
        # self.directions = [1, 1, 1, 1, 1, 1]

        # 🔹 Joint limits (radians)
        # self.limits = [
        #     (-math.pi, math.pi),
        #     (-math.pi/2, math.pi/2),
        #     (-math.pi, math.pi),
        #     (-math.pi, math.pi),
        #     (-math.pi, math.pi),
        #     (-math.pi, math.pi)
        # ]

        # 🔹 Thread safety for SPI
        self.lock = threading.Lock()

        # 🔹 Subscriber — joint states
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 🔹 Subscriber — gripper commands ("open" / "close")
        self.gripper_sub_ = self.create_subscription(
            String,
            '/gripper_command',
            self.gripper_callback,
            10
        )

        # 🔹 Communication handler (SPI → ESP)
        self.commHandler = Dispatcher().get_communication_handler("ESP")

        # 🔹 Timer (send every 100ms)
        self.timer = self.create_timer(0.1, self.send_steps)


    # Receive gripper commands ("open" / "close") and forward to ESP via SPI
    def gripper_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command == 'open':
            grip_val = 0
        elif command == 'close':
            grip_val = 1
        else:
            self.get_logger().warn(f"Unknown gripper command: '{command}' (expected 'open' or 'close')")
            return

        try:
            with self.lock:
                self.commHandler.sendData(['g', grip_val])
                self.get_logger().info(f"Gripper → {'Open' if grip_val == 0 else 'Close'}")
        except Exception as e:
            self.get_logger().error(f"Gripper SPI send failed: {e}")

    # Receive joint states
    def joint_state_callback(self, msg: JointState):
        joint_dict = dict(zip(msg.name, msg.position))  # fixed: was [msg.name](http://...)
        all_found = True
        for i, name in enumerate(self.joint_names):
            if name in joint_dict:
                self.current_joints[i] = joint_dict[name]
            else:
                all_found = False
        if all_found:
            self.joint_states_received = True

    # =========================================================
    #  Conversion: radians → steps
    # =========================================================
    def convert_to_steps(self, joints):
        steps = []
        for i, angle in enumerate(joints):
            # 🔹 Apply direction
            # angle *= self.directions[i]
            # 🔹 Apply offset
            # angle += self.offsets[i]
            # 🔹 Clamp limits
            # min_lim, max_lim = self.limits[i]
            # angle = max(min(angle, max_lim), min_lim)

            # 🔹 Convert to steps
            step = int((angle / (2 * math.pi)) * self.total_steps_per_rev)  #  fixed
            steps.append(step)
        return steps

    def send_steps(self):
        if not self.joint_states_received:
            self.get_logger().warn(
                "Waiting for /joint_states...",
                throttle_duration_sec=2.0
            )
            return

        steps = self.convert_to_steps(self.current_joints)
        steps.insert(0, 's')
        steps[6] = 0

        # Delay SPI send until the packet is stable; only final value is transmitted.
        if steps != self.pending_steps:
            self.pending_steps = steps.copy()
            self.stable_cycles = 0
            return

        self.stable_cycles += 1
        if self.stable_cycles < self.stability_threshold:
            return

        if steps == self.last_sent_steps:
            return

        self.last_sent_steps = steps.copy()

        try:
            with self.lock:
                self.commHandler.sendData(steps)
                self.get_logger().info(f"Sent steps: {steps}")
        except Exception as e:
            self.get_logger().error(f"SPI send failed: {e}")

# =============================================================
# 🚀 Main
# =============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Steppers()
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