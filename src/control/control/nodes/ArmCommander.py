#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.core.robot_state import RobotState
from geometry_msgs.msg import PoseStamped
import tf_transformations

from example_interfaces.msg import Bool
from example_interfaces.msg import Float64MultiArray
from my_robot_interfaces.msg import PoseCommand


ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot_description.srdf", {"name": "my_robot"})\
                                    .to_dict()

ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 5,  
            "planning_time": 5.0,  
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,  
            "max_acceleration_scaling_factor": 1.0  
        },
        "trajectory_execution": {
            "allowed_start_tolerance": 0.0,  
            "execution_duration_monitoring": False,  # Don't monitor execution timing
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1,
            "goal_joint_tolerance": 0.01,  
            "goal_position_tolerance": 0.03,  
            "goal_orientation_tolerance": 0.8  
        }
}

class ArmCommander(Node):
    def __init__(self):
        super().__init__("arm_commander")
        
       
        self.robot_ = MoveItPy(node_name="moveit_py", config_dict=ROBOT_CONFIG)
        self.arm_: PlanningComponent = self.robot_.get_planning_component("arm")
        self.gripper_: PlanningComponent = self.robot_.get_planning_component("gripper")

        self.open_gripper_sub_ = self.create_subscription(
            Bool, "open_gripper", self.open_gripper_callback, 10)
        
        self.joint_cmd_sub_ = self.create_subscription(
            Float64MultiArray, "joint_command", self.joint_command_callback, 10)
        
        self.pose_cmd_sub_ = self.create_subscription(
            PoseCommand, "pose_command", self.pose_command_callback, 10)

        self.get_logger().info("Commander node initialized")
        self.get_logger().info("Using MoveIt for both arm and gripper planning")

    def go_to_named_target(self, name):
        """Move to a named configuration from SRDF"""
        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(configuration_name=name)
        self.plan_and_execute_arm()

    def go_to_joints_target(self, joints):
        """Move to specific joint angles with tolerance"""
        if len(joints) != 6:
            self.get_logger().error(f"Expected 5 joints, got {len(joints)}")
            return
        
        robot_state = RobotState(self.robot_.get_robot_model())
        joint_values = {
            "Joint_1": joints[0],
            "Joint_2": joints[1],
            "Joint_3": joints[2],
            "Joint_4": joints[3],
            "Joint_5": joints[4],
            "Joint_6": joints[5],
        }
        robot_state.joint_positions = joint_values

        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(robot_state=robot_state)
        self.plan_and_execute_arm()

    def go_to_pose_target(self, x, y, z, roll, pitch, yaw):

        self.get_logger().info(
            f"go_to_pose_target  x={x:.3f} y={y:.3f} z={z:.3f} "
            f"roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}"
        )
        
        # Try MoveIt IK-based planning (1 attempt — fast fail)
        if self._try_pose_target(x, y, z, roll, pitch, yaw):
            return
        
        # IK failed → joint-space fallback (bypasses IK, uses MoveIt path planning only)
        self.get_logger().warn(
            "IK planning failed (5-DOF arm limitation). Using joint-space fallback..."
        )
        joints = self._approximate_joints_for_target(x, y, z)
        self.get_logger().info(f"Joint-space target: {[f'{j:.3f}' for j in joints]}")
        self.go_to_joints_target(joints)

    def _try_pose_target(self, x, y, z, roll, pitch, yaw):
        
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = q_x
        pose_goal.pose.orientation.y = q_y
        pose_goal.pose.orientation.z = q_z
        pose_goal.pose.orientation.w = q_w

        self.arm_.set_start_state_to_current_state()
        self.arm_.set_goal_state(pose_stamped_msg=pose_goal, pose_link="tool_link")
        return self.plan_and_execute_arm(max_attempts=1)

    def _approximate_joints_for_target(self, x, y, z):
        """Last-resort: compute approximate joint angles from known-working config.
        Based on pre_grasp [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0] which
        reaches roughly (0.50, 0.15, 0.06) in base_link frame."""
        import math

        REF_JOINTS = [-0.24, -0.20, -1.20, -0.48, 1.57, 0.0]
        REF_X, REF_Y, REF_Z = 0.50, 0.15, 0.06

        # Joint 1: base rotation toward target
        j1 = REF_JOINTS[0] + (math.atan2(y, x) - math.atan2(REF_Y, REF_X))
        j1 = max(-1.5707, min(1.5707, j1))

        # Reach scaling
        ref_reach = math.sqrt(REF_X**2 + REF_Y**2)
        target_reach = math.sqrt(x**2 + y**2)
        ratio = max(0.5, min(1.5, target_reach / ref_reach))

        j2 = REF_JOINTS[1] * ratio + (z - REF_Z) * 1.0
        j3 = REF_JOINTS[2] * ratio
        j4, j5, j6 = REF_JOINTS[3], REF_JOINTS[4], REF_JOINTS[5]

        joints = [j1, j2, j3, j4, j5, j6]
        return [max(-1.5707, min(1.5707, j)) for j in joints]


    def open_gripper(self):
        
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_open")
        self.plan_and_execute_gripper()

    def close_gripper(self):
        
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_closed")
        self.plan_and_execute_gripper()

    def half_close_gripper(self):
        
        self.gripper_.set_start_state_to_current_state()
        self.gripper_.set_goal_state(configuration_name="gripper_half_closed")
        self.plan_and_execute_gripper()

    def plan_and_execute_arm(self, max_attempts=3):
        
        for attempt in range(max_attempts):
            plan_result = self.arm_.plan()
            if plan_result:
                self.get_logger().info(f"Arm planning successful (attempt {attempt + 1}), executing...")
                try:
                    self.robot_.execute(plan_result.trajectory, controllers=["arm_controller"])
                    return True
                except Exception as e:
                    self.get_logger().warn(f"Execution failed: {e}, retrying planning...")
                    if attempt < max_attempts - 1:
                        
                        time.sleep(0.5)
                        continue
            else:
                if attempt < max_attempts - 1:
                    self.get_logger().warn(f"Planning failed (attempt {attempt + 1}/{max_attempts}), retrying...")
                    time.sleep(0.3)
        
        self.get_logger().error("Arm planning failed after all attempts!")
        return False

    def plan_and_execute_gripper(self):
        
        plan_result = self.gripper_.plan()
        if plan_result:
            self.get_logger().info("Gripper planning successful, executing...")
            self.robot_.execute(plan_result.trajectory, controllers=["gripper_controller"])
        else:
            self.get_logger().error("Gripper planning failed!")

    def open_gripper_callback(self, msg: Bool):
        
        if msg.data:
            self.open_gripper()
        else:
            self.close_gripper()

    def joint_command_callback(self, msg: Float64MultiArray):
        
        self.go_to_joints_target(msg.data)

    def pose_command_callback(self, msg: PoseCommand):
        
        self.go_to_pose_target(msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw)

def main(args=None):
    rclpy.init(args=args)
    node = ArmCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()