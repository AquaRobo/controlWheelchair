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
            "planning_attempts": 3,  
            "planning_time": 3.0,  
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 2.0,  
            "max_acceleration_scaling_factor": 2.0  
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
            "goal_orientation_tolerance": 0.4  
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
        
        success = self._try_pose_target(x, y, z, roll, pitch, yaw)
        
        if not success:
            self.get_logger().warn("Initial planning failed, trying with slight variations...")
            # Try with slight orientation variations to avoid singularities
            variations = [
                (0.05, 0.0, 0.0),  
                (-0.05, 0.0, 0.0),
                (0.0, 0.05, 0.0),  
                (0.0, -0.05, 0.0),
                (0.0, 0.0, 0.05),  
                (0.0, 0.0, -0.05),
                (0.03, 0.03, 0.0),  
                (-0.03, -0.03, 0.0),  
            ]
            
            for d_roll, d_pitch, d_yaw in variations:
                new_roll = roll + d_roll
                new_pitch = pitch + d_pitch
                new_yaw = yaw + d_yaw
                self.get_logger().info(f"Trying variation: roll={new_roll:.3f}, pitch={new_pitch:.3f}, yaw={new_yaw:.3f}")
                if self._try_pose_target(x, y, z, new_roll, new_pitch, new_yaw):
                    self.get_logger().info("Planning succeeded with variation!")
                    return
            
            self.get_logger().error("All planning attempts failed, including variations")
    
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
        return self.plan_and_execute_arm()

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

    def plan_and_execute_arm(self):
        
        max_attempts = 3
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