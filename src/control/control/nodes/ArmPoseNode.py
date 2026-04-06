#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from my_robot_interfaces.msg import PoseCommand
from tf_transformations import euler_from_quaternion
import threading
import time

# Known grab targets (x, y, z in base_link frame)
GRAB_TARGETS = {
    "coke_can":     {"x": 0.50, "y": 0.15, "z": 0.06},
    "red_cylinder": {"x": 0.22, "y": 0.12, "z": 0.20},
    "mustard":      {"x": 0.70, "y": 0.15, "z": 0.10},
    "default":      {"x": 0.50, "y": 0.15, "z": 0.06},  #
}

# Grasp orientation: end-effector pointing downward
GRASP_ROLL, GRASP_PITCH, GRASP_YAW = 0.0, 1.57, 0.0


GRAB_JOINTS = {
    "red_cylinder": {
        "pre_grasp": [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0],   
        "grasp":     [-0.24, -0.48, -0.84, -0.48, 1.57, 0.0],   
        "lift":      [-0.24, -0.20, -0.84, -0.48, 1.57, 0.0],    
    },
    "default": {
        "pre_grasp": [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0],
        "grasp":     [-0.24, -0.48, -0.84, -0.48, 1.57, 0.0],
        "lift":      [-0.24, -0.20, -0.84, -0.48, 1.57, 0.0],
    },
}


class ArmPoseNode(Node):
    def __init__(self):
        super().__init__('arm_pose_node')

        # Publishers
        self.joint_pub_ = self.create_publisher(JointTrajectory, 'arm_controller/joint_trajectory', 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, 'gripper_controller/joint_trajectory', 10)
        self.pose_pub_ = self.create_publisher(PoseCommand, 'pose_command', 10)

        # Subscribers
        self.command_sub_ = self.create_subscription(String, '/commanded_action', self.command_callback, 10)
        self.joint_state_sub_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.camera_target_sub_ = self.create_subscription(
            PointStamped,
            '/base_link/object_position',
            self.camera_target_callback,
            10
        )
        self.dynamic_target = None

        self.joint_names = ['Joint_1','Joint_2','Joint_3','Joint_4','Joint_5','Joint_6']
        self.current_joints = [0.0]*6
        self.joint_states_received = False
        self.step = 0.24

        # Predefined positions
        self.pos1 = self._quaternion_to_euler({"x":0.0109,"y":0.2385,"z":0.4557,"qx":0.1981,"qy":0.4774,"qz":0.7338,"qw":-0.44})
        self.pos2 = self._quaternion_to_euler({"x":0.1737,"y":-0.2474,"z":0.7361,"qx":0.2574,"qy":0.0742,"qz":0.9504,"qw":0.1574})

        # Start keyboard input thread
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

        self.get_logger().info(
            "Commander ready. Commands: up, down, left, right, forward, back, "
            "grab [can|cylinder|mustard], water, hand_up, hand_down, tilt_right, tilt_left, "
            "first position, second position, open, close, exit"
        )

    
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
    def camera_target_callback(self, msg: PointStamped):
        self.dynamic_target = {
            "x": msg.point.x,
            "y": msg.point.y,
            "z": msg.point.z,
            "roll": GRASP_ROLL,
            "pitch": GRASP_PITCH,
            "yaw": GRASP_YAW
        }
        self.get_logger().info(f"Target Updated from Camera -> X:{msg.point.x:.3f}, Y:{msg.point.y:.3f}, Z:{msg.point.z:.3f}")

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

    def command_callback(self, msg: String):
        command = msg.data.lower()
        threading.Thread(target=self.handle_command, args=(command,), daemon=True).start()

    # ----------------- Command Handling -----------------
    def handle_command(self, command):
        
        if command in ["grab", "grab default"]:
            self.grab_sequence("default")
        elif command == "water":
            if self.dynamic_target is not None:
                self.get_logger().info("Moving to dynamic 'water' target detected by camera...")
                self.move_to_pose(self.dynamic_target)
            else:
                self.get_logger().warn("Camera target ('water') is unknown! Wait for /base_link/object_position.")
        elif command in ["first position", "pos1"]:
            self.move_to_pose(self.pos1)
        elif command in ["second position", "pos2"]:
            self.move_to_pose(self.pos2)
    
        elif command == "open":
            self.publish_gripper(True)
        elif command == "close":
            self.publish_gripper(False)
        elif command in ["up","down","left","right","forward","back","hand up","hand down","tilt right","tilt left"]:
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
        elif command=="hand down": 
            delta[4]-=self.step
        elif command=="hand up": 
            delta[4]+=self.step
        elif command=="tilt right": 
            delta[3]+=self.step
        elif command=="tilt left": 
            delta[3]-=self.step
        elif command=="left": 
            delta[0]+=self.step
        elif command=="right": 
            delta[0]-=self.step
        elif command=="forward": 
            delta[1]-=self.step*0.7
            delta[2]+=self.step*0.5
        elif command=="back": 
            delta[1]+=self.step*0.7
            delta[2]-=self.step*0.5
        new_joints = [c+d for c,d in zip(self.current_joints, delta)]
        self.publish_joints(new_joints, duration=0.15)

    
    def grab_sequence(self, target_name="default"):
        """Joint-angle-based grab sequence.
        Uses direct joint commands — no MoveIt IK needed.
        """
        joints = GRAB_JOINTS.get(target_name, GRAB_JOINTS["default"])

        self.get_logger().info(f"Starting grab sequence for '{target_name}'...")

        # Step 0: Move to home position
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Step 0: Moving to home position...")
        self.publish_joints(home_joints, duration=1.0)
        self.wait_for_joints(home_joints, tol=0.1, timeout=4.0)
        time.sleep(0.5)

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

        # Step 3: Move to pre-grasp (above object)
        self.get_logger().info("Step 3: Moving to pre-grasp...")
        self.publish_joints(joints["pre_grasp"], duration=1.0)
        self.wait_for_joints(joints["pre_grasp"], tol=0.1, timeout=4.0)
        time.sleep(0.5)

        # Step 4: Descend to grasp
        self.get_logger().info("Step 4: Descending to grasp...")
        self.publish_joints(joints["grasp"], duration=1.0)
        self.wait_for_joints(joints["grasp"], tol=0.1, timeout=4.0)
        time.sleep(0.7)

        # Step 5: Close gripper
        self.get_logger().info("Step 5: Closing gripper...")
        self.publish_gripper(False, duration=1.0)
        time.sleep(1.5)

        # Step 6: Lift straight up
        self.get_logger().info("Step 6: Lifting object straight up...")
        self.publish_joints(joints["lift"], duration=1.8)
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
        rclpy.shutdown()


if __name__=="__main__":
    main()