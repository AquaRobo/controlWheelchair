import rclpy
import math
import threading
from rclpy.node import Node
from sensor_msgs.msg import JointState
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

        # 🔹 Motor configuration
        self.steps_per_rev = 200       # 1.8° stepper
        self.microstepping = 16        # driver setting
        self.gear_ratio = 1            # adjust if gearbox exists
        self.total_steps_per_rev = (   # ✅ fixed: was [self.total](http://...)
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

        # 🔹 Subscriber
        self.joint_state_sub_ = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # 🔹 Communication handler (SPI → ESP)
        self.commHandler = Dispatcher().get_communication_handler("SENSOR")

        # 🔹 Timer (send every 100ms)
        self.timer = self.create_timer(0.1, self.send_steps)

    # 📥 Receive joint states
    def joint_state_callback(self, msg: JointState):
        joint_dict = dict(zip(msg.name, msg.position))  # ✅ fixed: was [msg.name](http://...)
        all_found = True
        for i, name in enumerate(self.joint_names):
            if name in joint_dict:
                self.current_joints[i] = joint_dict[name]
            else:
                all_found = False
        if all_found:
            self.joint_states_received = True

    # =========================================================
    # ⚙️ Conversion: radians → steps
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
            step = int((angle / (2 * math.pi)) * self.total_steps_per_rev)  # ✅ fixed
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
        steps[6]=0

        try:
            with self.lock:
                self.commHandler.sendData(steps)
                self.get_logger().info(f"Sent steps: {steps}")  # ✅ fixed: removed broken self._logger line
        except Exception as e:
            self.get_logger().error(f"SPI send failed: {e}")

# =============================================================
# 🚀 Main
# =============================================================
def main(args=None):
    rclpy.init(args=args)
    node = Steppers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()