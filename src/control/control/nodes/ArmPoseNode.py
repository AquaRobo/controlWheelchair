#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from my_robot_interfaces.msg import ObjectDetection
from builtin_interfaces.msg import Duration
from my_robot_interfaces.msg import PoseCommand
from tf_transformations import euler_from_quaternion
import threading
import time

GRAB_TARGETS = {
    "coke_can":     {"x": 0.50, "y": 0.15, "z": 0.06},
    "red_cylinder": {"x": 0.22, "y": 0.12, "z": 0.20},
    "mustard":      {"x": 0.70, "y": 0.15, "z": 0.10},
    "default":      {"x": 0.50, "y": 0.15, "z": 0.06},  
}

GRASP_ROLL, GRASP_PITCH, GRASP_YAW = 0.0, 1.57, 0.0

Z_MIN = 0.12
Z_PRE_GRASP_OFFSET = 0.04

# Home position: exact physical values recorded from the real robot
HOME_JOINTS = [-9.306237189323058e-05, 0.73, 0.49999999998185113, -0.4999999999946486, 1.0, 0.0]


GRAB_JOINTS = {
    "red_cylinder": {
        "pre_grasp": [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0],   
        "grasp":     [-0.24, -0.48, -0.84, -0.48, 1.57, 0.0],   
        "lift":      [-0.24, -0.20, -0.84, -0.48, 1.57, 0.0],    
    },
    "default": {
        "pre_grasp": [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0],
        "grasp":     [-0.24, -0.56, -0.84, -0.48, 1.57, 0.0],
        "lift":      [-0.24, -0.20, -0.84, -0.48, 1.57, 0.0],
    },
}


GRIPPER_OPEN = [0.05, -0.05]
GRIPPER_CLOSED = [1.45, -1.40]


class ArmPoseNode(Node):
    def __init__(self):
        super().__init__('arm_pose_node')

        # Publishers
        self.joint_pub_ = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.real_gripper_pub_ = self.create_publisher(String, '/gripper_command', 10)
        self.pose_pub_ = self.create_publisher(PoseCommand, 'pose_command', 10)

        # Subscribers
        self.command_sub_ = self.create_subscription(String, '/commanded_action', self.command_callback, 10)
        self.robot_sub_ = self.create_subscription(String, '/commanded_robot', self.robot_callback, 10)
        self._commanded_robot = None  # Track which robot was last commanded
        self.joint_state_sub_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.camera_target_sub_ = self.create_subscription(
            ObjectDetection,
            '/base_link/object_detection',
            self.camera_target_callback,
            10
        )
        self.dynamic_target = None

        self.joint_names = ['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Joint_6']
        self.current_joints = [0.0]*6
        self.joint_states_received = False
        self.gripper_joint_names = ['Gripper_Servo_Gear_Joint', 'Gripper_Idol_Gear_Joint']
        self.current_gripper = [0.0, 0.0]
        self.gripper_state_received = False

        self.step = 0.5
        self.busy = False  

        # Predefined positions
        self.pos1 = self._quaternion_to_euler({"x":0.0109,"y":0.2385,"z":0.4557,"qx":0.1981,"qy":0.4774,"qz":0.7338,"qw":-0.44})
        self.pos2 = self._quaternion_to_euler({"x":0.1737,"y":-0.2474,"z":0.7361,"qx":0.2574,"qy":0.0742,"qz":0.9504,"qw":0.1574})

        # Automatic startup homing — runs on real hardware AND simulation
        self.startup_done = False
        self.startup_timer = self.create_timer(1.0, self.startup_home)

        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info(
            "Commander ready. Commands: up, down, left, right, forward, back, "
            "grab [can|cylinder|mustard], water, hand up, hand down, tilt right, tilt left, "
            "first position, second position, open, close, exit"
        )

    
    def startup_home(self):
        if not self.startup_done and self.joint_states_received:
            self.get_logger().info("Simulation started. Commanding robot to straight up initial position...")
            # Command straight up based on user requested Joint_2 and Joint_3 angles
            home_joints = [-9.306237189323058e-05, 0.73, 0.49999999998185113, -0.4999999999946486, 1.0, 0.0]
            self.publish_joints(home_joints, duration=2.5)
            self.startup_done = True
            if hasattr(self, 'startup_timer') and self.startup_timer:
                self.startup_timer.cancel()

    def _quaternion_to_euler(self, pose):
        roll, pitch, yaw = euler_from_quaternion([pose["qx"], pose["qy"], pose["qz"], pose["qw"]])
        return {"x":pose["x"], "y":pose["y"], "z":pose["z"], "roll":roll, "pitch":pitch, "yaw":yaw}

    def wait_for_joints(self, target_positions, tol=0.12, timeout=4.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            errors = [abs(a - b) for a, b in zip(self.current_joints, target_positions)]
            if max(errors) < tol:
                return True
            time.sleep(0.01)
        return False

    # ----------------- Subscribers -----------------
    def camera_target_callback(self, msg: ObjectDetection):
        if msg.is_detected and msg.object_name.lower() in ["bottle", "cup"]:
            self.dynamic_target = {
                "x": msg.object_pose.x,
                "y": msg.object_pose.y,
                "z": msg.object_pose.z,
                "roll": GRASP_ROLL,
                "pitch": GRASP_PITCH,
                "yaw": GRASP_YAW
            }
       

    def joint_state_callback(self, msg: JointState):
        joint_dict = dict(zip(msg.name, msg.position))

        all_found = True
        for i, name in enumerate(self.joint_names):
            if name in joint_dict:
                self.current_joints[i] = joint_dict[name]
            else:
                all_found = False
        if all_found:
            self.joint_states_received = True

     
        gripper_found = True
        for i, name in enumerate(self.gripper_joint_names):
            if name in joint_dict:
                self.current_gripper[i] = joint_dict[name]
            else:
                gripper_found = False
        if gripper_found:
            self.gripper_state_received = True

    def robot_callback(self, msg: String):
        self._commanded_robot = msg.data.upper().strip()
        self.get_logger().info(f"Commanded robot: {self._commanded_robot}")

    def command_callback(self, msg: String):
        # Only respond to commands when Jarvis is the active robot
        if self._commanded_robot != "JARVIS":
            return
        command = msg.data.lower()
        threading.Thread(target=self.handle_command, args=(command,), daemon=True).start()

    # ----------------- Command Handling -----------------
    def handle_command(self, command):
        if self.busy:
            self.get_logger().warn(f"Busy — ignoring command: '{command}'")
            return

        if command in ["grab", "grab default"]:
            self.grab_sequence("default")
        elif command in ["water", "cup", "bottle"]:
            threading.Thread(target=self.water_scan_sequence, daemon=True).start()
        elif command in ["first position", "pos1"]:
            self.move_to_pose(self.pos1)
        elif command in ["second position", "pos2"]:
            self.move_to_pose(self.pos2)
    
        elif command == "open":
            self.publish_gripper(True)
        elif command == "close":
            self.publish_gripper(False)
        elif command in ["up","down","left","right","forward","back","hand up","hand down","tilt right","tilt left","lean","level","raise","lower"]:
            self.incremental_move(command)
        elif command == "exit":
            self.get_logger().info("Exiting...")
            rclpy.shutdown()
        else:
            self.get_logger().warn(f"Unknown command: {command}")

    # ----------------- Pose movement -----------------
    def move_to_pose(self, target):
        msg = PoseCommand()
        msg.x, msg.y, msg.z = target["x"], target["y"], target["z"]
        msg.roll, msg.pitch, msg.yaw = target["roll"], target["pitch"], target["yaw"]
        self.pose_pub_.publish(msg)

    # ----------------- Joint publishing -----------------
    def publish_joints(self, positions, duration=0.3):
        positions = list(positions)
        positions[5] = 0.0  # 5-DOF arm: Joint_6 is always zero on real hardware
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration-int(duration))*1e9))
        msg.points.append(point)
        self.joint_pub_.publish(msg)

    def publish_gripper(self, open_gripper, duration=0.2):
        
        servo = 0.05 if open_gripper else 1.45   # Max closed is ~1.55
        idol = -0.05 if open_gripper else -1.40  # Max closed is ~-1.49
        msg = JointTrajectory()
        msg.joint_names = ['Gripper_Servo_Gear_Joint', 'Gripper_Idol_Gear_Joint']
        point = JointTrajectoryPoint()
        point.positions = [servo, idol]
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration-int(duration))*1e9))
        msg.points.append(point)
        self.gripper_pub_.publish(msg)

        # Publish to real hardware topic
        grip_msg = String()
        grip_msg.data = 'open' if open_gripper else 'close'
        self.real_gripper_pub_.publish(grip_msg)

    # ----------------- Incremental move -----------------
    def incremental_move(self, command):
        if not self.joint_states_received:
            self.get_logger().warn("Waiting for joint states...")
            return
        delta = [0.0]*6
        if command=="up": 
            delta[2]+=self.step
        elif command=="down": 
            delta[2]-=self.step
        elif command=="hand down" or command=="lower": 
            delta[4]-=self.step
        elif command=="hand up" or command=="raise": 
            delta[4]+=self.step
        elif command=="tilt right" or command=="lean": 
            delta[3]+=self.step
        elif command=="tilt left" or command=="level": 
            delta[3]-=self.step
        elif command=="left": 
            delta[0]+=self.step
        elif command=="right": 
            delta[0]-=self.step
        elif command=="forward": 
            delta[1]-=self.step*0.7
            # delta[2]+=self.step*0.5
        elif command=="back": 
            delta[1]+=self.step*0.7
            # delta[2]-=self.step*0.5
        new_joints = [c+d for c,d in zip(self.current_joints, delta)]
        self.publish_joints(new_joints, duration=0.15)

    def _joints_to_steps(self, joints):
        """Convert joint angles to integer stepper steps (matches ArmControl.py logic)."""
        import math
        steps_per_rev = 200
        microstepping = 16
        total_steps_per_rev = steps_per_rev * microstepping
        return [int((a / (2 * math.pi)) * total_steps_per_rev) for a in joints]

    def wait_for_steps_stable(self, timeout=3.0, stable_duration=0.45):
        """Wait until /joint_states produce identical integer step values for
        `stable_duration` seconds.  This satisfies the Steppers node stability
        filter (3× 100 ms cycles of identical step packets).

        Returns True when stable, False on timeout.
        """
        stable_start = None
        last_steps = None
        deadline = time.time() + timeout

        while time.time() < deadline:
            time.sleep(0.05)
            current_steps = self._joints_to_steps(self.current_joints)
            if current_steps == last_steps:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= stable_duration:
                    return True
            else:
                last_steps = current_steps
                stable_start = None

        return False

    def wait_for_arm_to_settle(self, timeout=25.0, settle_time=2.0):
        """Waits dynamically until the arm joint velocities are effectively zero."""
        start_joints = list(self.current_joints)
        
        # 1. Wait until arm actually initiates physical movement
        wait_start = time.time()
        has_moved = False
        while time.time() - wait_start < 8.0:
            time.sleep(0.2)
            diff = max(abs(a - b) for a, b in zip(self.current_joints, start_joints))
            if diff > 0.02:
                has_moved = True
                break
                
        if not has_moved:
            self.get_logger().warn("Arm never moved! MoveIt MK planning likely failed.")
            return False

        # 2. Now that it is moving, monitor for it to stop (settle)
        # We check velocity over a 0.5 second window to accurately catch slow-moving simulated arms
        start_t = time.time()
        last_joints = list(self.current_joints)
        settled_start = None

        while time.time() - start_t < timeout:
            time.sleep(0.5)
            diff = max(abs(a - b) for a, b in zip(self.current_joints, last_joints))
            last_joints = list(self.current_joints)

            if diff < 0.015:  
                if settled_start is None:
                    settled_start = time.time()
                elif time.time() - settled_start >= settle_time:
                    return True
            else:
                settled_start = None
        return False

    def wait_for_gripper_to_settle(self, target_positions, tol=0.08, timeout=6.0,
                                    settle_time=0.5, vel_tol=0.01, require_reach=False):
        
        if not self.gripper_state_received:
            self.get_logger().warn(
                "No gripper joint state feedback received — falling back to fixed wait."
            )
            time.sleep(timeout)
            return False

        start_pos = list(self.current_gripper)
        kickoff_start = time.time()
        while time.time() - kickoff_start < min(1.5, timeout):
            time.sleep(0.1)
            if max(abs(a - b) for a, b in zip(self.current_gripper, start_pos)) > 0.02:
                break

        start_t = time.time()
        last_pos = list(self.current_gripper)
        settled_start = None

        while time.time() - start_t < timeout:
            time.sleep(0.2)
            diff = max(abs(a - b) for a, b in zip(self.current_gripper, last_pos))
            last_pos = list(self.current_gripper)

            if require_reach:
                reached = all(abs(a - b) < tol for a, b in zip(self.current_gripper, target_positions))
            else:
                reached = True  # don't require hitting the exact closed value (object blocks it)

            if reached and diff < vel_tol:
                if settled_start is None:
                    settled_start = time.time()
                elif time.time() - settled_start >= settle_time:
                    return True
            else:
                settled_start = None

        self.get_logger().warn("Gripper did not settle within timeout, continuing anyway.")
        return False
    
    def water_scan_sequence(self):
        self.get_logger().info("Starting water scan sequence...")
        self.busy = True
        try:
            self.dynamic_target = None
            direction = 1
            
            # Step 0: Move to home position
            home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.get_logger().info("Step 0: Moving to home position...")
            self.publish_joints(home_joints, duration=1.0)
            self.wait_for_joints(home_joints, tol=0.1, timeout=4.0)
            time.sleep(0.5)
            
            scan_joints = list(self.current_joints)

            scan_levels = [
                {"duration": 20.0, "joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "name": "mid"},
                {"duration": 20.0, "joints": [0.0, 0.0, 0.5, 0.0, 0.0, 0.0], "name": "upper"},
                {"duration": 15.0, "joints": [0.0, -0.20, -1.20, -0.48, 1.57, 0.0], "name": "lower"}
            ]

            for level in scan_levels:
                if self.dynamic_target is not None:
                    break

                self.get_logger().info(f"Scanning at {level['name']} level for {level['duration']} seconds...")
                
                target_base = list(level["joints"])
                target_base[0] = scan_joints[0]
                self.publish_joints(target_base, duration=1.0)
                self.wait_for_joints(target_base, tol=0.1, timeout=4.0)
                scan_joints = list(target_base)
                
                level_start_time = time.time()
                while time.time() - level_start_time < level["duration"]:
                    if self.dynamic_target is not None:
                        break

                    scan_joints[0] += direction * 0.2

                    if scan_joints[0] >= 1.2:
                        direction = -1
                        scan_joints[0] = 1.2
                    elif scan_joints[0] <= -1.0:
                        direction = 1
                        scan_joints[0] = -1.0

                    self.publish_joints(scan_joints, duration=0.5)
                    self.wait_for_joints(scan_joints, tol=0.05, timeout=2.0)
                    self.wait_for_steps_stable(timeout=2.0, stable_duration=0.45)

            if self.dynamic_target is None:
                self.grab_sequence("red_cylinder", skip_home=True, start_from_pregrasp=True)
                return

            time.sleep(1.5)

            self.get_logger().info("Executing IK to target...")
            self.move_to_pose(self.dynamic_target)
            self.get_logger().info("Waiting dynamically for the arm to physically reach the target...")
            self.wait_for_arm_to_settle(timeout=15.0, settle_time=1.5)

            self.get_logger().info("Closing gripper...")
            self.publish_gripper(False, duration=1.5)
            self.get_logger().info("Waiting dynamically for gripper to firmly grasp...")
            self.wait_for_gripper_to_settle(
                GRIPPER_CLOSED, tol=0.08, timeout=16.0, settle_time=1.5
            )
            self.get_logger().info("Holding to let the physical gripper finish closing...")
            time.sleep(3.0)

            right_joints = [-1.2, -0.20, -1.20, -0.48, 1.57, 0.0]
            self.get_logger().info("Moving to final right position...")
            self.publish_joints(right_joints, duration=3.0)
            self.wait_for_arm_to_settle(timeout=15.0, settle_time=1.0)

            self.get_logger().info("Water sequence complete!")
        finally:
            self.busy = False

    def grab_sequence(self, target_name="default", skip_home=False, start_from_pregrasp=False):
        """Joint-angle-based grab sequence.
        Uses direct joint commands — no MoveIt IK needed.
        """
        joints = GRAB_JOINTS.get(target_name, GRAB_JOINTS["default"])

        self.get_logger().info(f"Starting grab sequence for '{target_name}'...")
        self.busy = True

        try:
            if not skip_home and not start_from_pregrasp:
                # Step 0: Move to home position (straight up)
                home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.get_logger().info("Step 0: Moving to home position...")
                self.publish_joints(home_joints, duration=1.0)
                self.wait_for_joints(home_joints, tol=0.1, timeout=4.0)
                time.sleep(0.5)

            if not start_from_pregrasp:
                # Step 1: Open gripper
                self.get_logger().info("Step 1: Opening gripper...")
                self.publish_gripper(True, duration=0.5)
                time.sleep(1.0)

                # Step 2: Hover high up (safe position to avoid sweeping through objects)
                safe_hover = [0.0, -0.6, -1.2, 0.0, 1.57, 0.0]
                self.get_logger().info("Step 2: Moving to safe hover...")
                self.publish_joints(safe_hover, duration=1.0)
                self.wait_for_joints(safe_hover, tol=0.1, timeout=4.0)
                time.sleep(0.5)
            else:
                self.get_logger().info("Skipping Steps 0-2. Ensuring gripper is open...")
                self.publish_gripper(True, duration=0.5)
                time.sleep(0.5)

                self.get_logger().info("Intermediate step: Moving left slightly before pre-grasp...")
                clearance_joints = list(joints["pre_grasp"])
                clearance_joints[0] += 0.40  # Move Joint_1 further left
                
                self.publish_joints(clearance_joints, duration=1.0)
                self.wait_for_joints(clearance_joints, tol=0.1, timeout=3.0)

            # Step 3: Move to pre-grasp (above object)
            self.get_logger().info("Step 3: Moving to pre-grasp...")
            self.publish_joints(joints["pre_grasp"], duration=1.0)
            self.wait_for_joints(joints["pre_grasp"], tol=0.1, timeout=3.5)
            time.sleep(0.4)

            # Step 4: Descend to grasp
            self.get_logger().info("Step 4: Descending to grasp...")
            self.publish_joints(joints["grasp"], duration=0.5)
            self.wait_for_joints(joints["grasp"], tol=0.1, timeout=3.5)
            time.sleep(0.1)

            # Step 5: Close gripper
            self.get_logger().info("Step 5: Closing gripper...")
            self.publish_gripper(False, duration=0.6)
            time.sleep(1.0)

            # Step 6: Lift straight up
            self.get_logger().info("Step 6: Lifting object straight up...")
            self.publish_joints(joints["lift"], duration=3.0)
            self.wait_for_joints(joints["lift"], tol=0.1, timeout=4.0)
            time.sleep(0.5)

            POS2_JOINTS = [
                -0.7945954866228031,
                0.7620747480470138,
                -0.16752820576785488,
                -0.13463619948432545,
                0.345653063868557,
                5.1772133779046094e-05
            ]

            self.get_logger().info("Step 7: Moving to pos2 (fast joint move)...")
            self.publish_joints(POS2_JOINTS, duration=1.0)
            self.wait_for_joints(POS2_JOINTS, tol=0.1, timeout=3.0)

            self.get_logger().info(f"Grab sequence for {target_name} complete and moved to pos2!")
        finally:
            self.busy = False

    
    def keyboard_loop(self):
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").lower()
            except EOFError:
                cmd = "exit"
            self.handle_command(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArmPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()