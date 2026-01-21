#!/usr/bin/env python3

"""
MoveIt2 Python API script for RTOP bimanual robot.
Commands cartesian poses for the gripper, closes gripper, then moves to another pose.

Usage:
    1. Start the simulation and MoveIt:
       ros2 launch rtop_moveit_config demo.launch.py
    
    2. Run this script:
       python3 src/i2rt/bimanual/rtop_moveit_config/scripts/moveit_py_cartesian_demo.py

IMPORTANT: The launch file must include these planning_scene_monitor_parameters:
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,  # Required!
        "publish_robot_description_semantic": True,  # Required!
    }
    
Note: The demo.launch.py should already include these parameters.
"""

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import os
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import traceback
import time
import yaml


def main():
    rclpy.init()
    node = Node("moveit_py_cartesian_demo")
    logger = node.get_logger()

    try:
        # Get use_sim_time parameter
        if node.has_parameter('use_sim_time'):
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value
        else:
            node.declare_parameter('use_sim_time', True)
            use_sim_time = node.get_parameter('use_sim_time').get_parameter_value().bool_value

        logger.info("MoveIt2 Python API Cartesian Demo for RTOP")
        logger.info(f"use_sim_time: {use_sim_time}")

        # Configuration
        arm_planning_group = "right_arm"  # or "right_arm"
        gripper_planning_group = "right_gripper"  # or "right_gripper"
        end_effector_link = "right_link_6"  # or "right_link_6"
        gripper_action_topic = "/right_gripper_controller/gripper_cmd"  # or "/right_gripper_controller/gripper_cmd"
        base_frame = "base"  # Base frame for poses

        logger.info(f"Initializing MoveItPy for planning group: {arm_planning_group}")

        # Build MoveIt configuration
        moveit_config_builder = MoveItConfigsBuilder(
            robot_name="rtop_calib",
            package_name="rtop_moveit_config"
        )

        # CRITICAL: Set the path to moveit_planning_python.yaml manually
        # This ensures planning pipelines are loaded correctly
        # See: https://github.com/ros-planning/moveit2/issues/2409
        # Based on working example from xarm_scripts
        moveit_planning_python_path_install = os.path.join(
            get_package_share_directory("rtop_moveit_config"),
            "config",
            "moveit_planning_python.yaml"
        )
        # Fallback to source directory if install doesn't exist
        moveit_planning_python_path_source = os.path.join(
            os.path.dirname(__file__),
            "..",
            "config",
            "moveit_planning_python.yaml"
        )
        moveit_planning_python_path_source = os.path.abspath(moveit_planning_python_path_source)
        
        if os.path.exists(moveit_planning_python_path_install):
            moveit_planning_python_path = moveit_planning_python_path_install
        elif os.path.exists(moveit_planning_python_path_source):
            moveit_planning_python_path = moveit_planning_python_path_source
        else:
            logger.error(f"moveit_planning_python.yaml not found in install ({moveit_planning_python_path_install}) or source ({moveit_planning_python_path_source})")
            logger.error("Please build the package: colcon build --packages-select rtop_moveit_config")
            raise FileNotFoundError(f"moveit_planning_python.yaml not found")
        
        # Set the moveit_cpp/planning_python config file path
        moveit_config_builder = moveit_config_builder.moveit_cpp(file_path=moveit_planning_python_path)
        logger.info(f"Using moveit_planning_python.yaml from: {moveit_planning_python_path}")

        # Get config dict directly (like the working xarm example)
        # Don't use to_moveit_configs() - use to_dict() directly
        moveit_config_dict = moveit_config_builder.to_dict()
        
        # Debug: Check if planning_pipelines are in config_dict
        if 'planning_pipelines' in moveit_config_dict:
            pipelines = moveit_config_dict['planning_pipelines']
            if isinstance(pipelines, dict):
                logger.info(f"✓ Planning pipelines in config_dict: {list(pipelines.keys())}")
            else:
                logger.info(f"✓ Planning pipelines in config_dict: {type(pipelines)}")
        else:
            logger.warn("WARNING: planning_pipelines NOT in config_dict!")

        # Add use_sim_time to config dict for MoveItPy's internal node if not already present
        if 'use_sim_time' not in moveit_config_dict:
            moveit_config_dict['use_sim_time'] = use_sim_time
        
        # Apply QoS override fix (needed after adding use_sim_time)
        # This is from the working xarm example
        try:
            MoveItConfigsBuilder._add_qos_overrides_for_sim_time(moveit_config_dict)
            logger.info("✓ Applied QoS overrides for sim_time")
        except Exception as e:
            logger.warn(f"Could not apply QoS overrides: {e}")

        logger.info("MoveIt configuration loaded")

        # IMPORTANT: MoveItPy needs planning pipelines
        # They should be in the config_dict, but if not, MoveItPy will try to load from parameter server
        # The demo.launch.py must be running to provide these parameters if they're not in config_dict
        logger.info("Initializing MoveItPy (this may take a moment)...")
        logger.info("Note: If planning pipelines are not in config, MoveItPy will load from parameter server")
        logger.info("Make sure 'ros2 launch rtop_moveit_config demo.launch.py' is running if needed!")
        
        # Wait a bit for services to be available
        time.sleep(2.0)

        try:
            # Initialize MoveItPy
            # MoveItPy will look for planning pipelines in:
            # 1. The config_dict (if provided) - we've tried to add them
            # 2. The parameter server - it looks for /move_group/planning_pipelines by default
            # Since demo.launch.py is running, the planning pipelines are on parameter server
            # MoveItPy should be able to find them automatically
            logger.info("Attempting to initialize MoveItPy...")
            logger.info("Planning pipelines should be available from /move_group node on parameter server")
            logger.info("If this fails, the planning pipelines may not be in the expected format")
            
            # MoveItPy creates its own node internally and looks for planning pipelines
            # Initialize MoveItPy (like the working xarm example)
            logger.info("Initializing MoveItPy with config_dict...")
            rtop_moveit = MoveItPy(node_name="moveit_py", config_dict=moveit_config_dict)
        except RuntimeError as e:
            if "planning pipelines" in str(e).lower():
                logger.error(f"Failed to load planning pipelines: {e}")
                logger.error("")
                logger.error("SOLUTION: You need to run the launch file first:")
                logger.error("  ros2 launch rtop_moveit_config demo.launch.py")
                logger.error("")
                logger.error("The launch file loads the planning pipelines into the parameter server.")
                logger.error("Wait for the launch file to fully initialize, then run this script again.")
            else:
                logger.error(f"Failed to initialize MoveItPy: {e}")
            raise
        except Exception as e:
            logger.error(f"Failed to initialize MoveItPy: {e}")
            logger.error("Make sure 'ros2 launch rtop_moveit_config demo.launch.py' is running!")
            raise
        arm_planner = rtop_moveit.get_planning_component(arm_planning_group)
        gripper_planner = rtop_moveit.get_planning_component(gripper_planning_group)
        logger.info("✓ MoveItPy initialized")

        # Initialize gripper action client
        gripper_action_client = ActionClient(node, GripperCommand, gripper_action_topic)
        logger.info(f"Waiting for gripper action server: {gripper_action_topic}")
        if not gripper_action_client.wait_for_server(timeout_sec=10.0):
            logger.error(f"Gripper action server not available!")
            return 1
        logger.info("✓ Gripper action server available")

        # ==========================================
        # Step 0: Open gripper (start with open gripper)
        # ==========================================
        logger.info("\n=== Step 0: Opening gripper ===")
        
        # Use action interface to open gripper
        gripper_goal_open = GripperCommand.Goal()
        gripper_goal_open.command.position = 0.04  # Open position (0.04 = open, 0.0 = closed)
        gripper_goal_open.command.max_effort = 50.0
        
        logger.info("Sending gripper open command...")
        send_goal_future = gripper_action_client.send_goal_async(gripper_goal_open)
        rclpy.spin_until_future_complete(node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if goal_handle.accepted:
            logger.info("Gripper goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            result = result_future.result().result
            logger.info(f"✓ Gripper opened (position: {result.position})")
        else:
            logger.warn("Gripper action goal rejected, trying MoveIt planning...")
            # Fallback: Use MoveItPy to set gripper to open state
            gripper_planner.set_start_state_to_current_state()
            
            robot_model = rtop_moveit.get_robot_model()
            gripper_state = RobotState(robot_model)
            
            # Set gripper joints to open position
            # Based on SRDF: right_open has right_left_finger=0.037524, right_right_finger=-0.037524
            gripper_state.set_joint_positions("right_left_finger", [0.037524])
            gripper_state.set_joint_positions("right_right_finger", [-0.037524])
            
            gripper_planner.set_goal_state(robot_state=gripper_state)
            
            gripper_plan_result = gripper_planner.plan()
            if gripper_plan_result:
                gripper_trajectory = gripper_plan_result.trajectory
                rtop_moveit.execute(gripper_planning_group, gripper_trajectory, blocking=True)
                logger.info("✓ Gripper opened via MoveIt")
            else:
                logger.error("Failed to open gripper!")

        time.sleep(1.0)

        # ==========================================
        # Step 1: Move to first cartesian pose
        # ==========================================
        logger.info("\n=== Step 1: Moving to first cartesian pose ===")
        
        # Set start state to current state
        arm_planner.set_start_state_to_current_state()

        # Try a pose that should be within the robot's workspace
        # For a bimanual robot with arms at y=±0.305m and z=1.0m above base:
        # - Left arm can reach positions with y > 0
        # - Right arm can reach positions with y < 0
        # - Both arms can reach forward (x > 0) and upward (z > 0)
        pose_goal_1 = PoseStamped()
        pose_goal_1.header.frame_id = base_frame
        pose_goal_1.pose.position.x = 0.4   # Forward from base
        pose_goal_1.pose.position.y = 0.0   # Slightly to the right (for right arm, negative y)
        pose_goal_1.pose.position.z = 1.15   # Slightly above arm base (1.0 + 0.15)
        
        # Use a simple orientation that should be reachable
        # Pointing forward and slightly down (common for pick and place
        pose_goal_1.pose.orientation.x = 0.0
        pose_goal_1.pose.orientation.y = 0.707
        pose_goal_1.pose.orientation.z = 0.0
        pose_goal_1.pose.orientation.w = 0.707  # 90 degrees around y-axis

        logger.info(f"Setting goal pose in frame '{base_frame}':")
        logger.info(f"  Position: x={pose_goal_1.pose.position.x}, y={pose_goal_1.pose.position.y}, z={pose_goal_1.pose.position.z}")
        logger.info(f"  Orientation: w={pose_goal_1.pose.orientation.w}, x={pose_goal_1.pose.orientation.x}, y={pose_goal_1.pose.orientation.y}, z={pose_goal_1.pose.orientation.z}")
        logger.info(f"  Target link: {end_effector_link}")

        arm_planner.set_goal_state(pose_stamped_msg=pose_goal_1, pose_link=end_effector_link)
        
        logger.info("Planning to goal pose...")
        plan_result = arm_planner.plan()
        if not plan_result:
            logger.error("Failed to plan to first pose!")
            logger.error("Possible reasons:")
            logger.error("  1. Goal pose is outside robot workspace")
            logger.error("  2. Goal pose is in collision")
            logger.error("  3. Goal orientation is not reachable")
            logger.error("  4. Current robot state is invalid")
            logger.error("")
            logger.error("Try adjusting the goal pose coordinates or orientation")
            return 1

        robot_trajectory = plan_result.trajectory
        logger.info("✓ Plan found, executing...")
        rtop_moveit.execute(arm_planning_group, robot_trajectory, blocking=True)
        logger.info("✓ Reached first pose")

        # Wait a bit
        time.sleep(1.0)

        # ==========================================
        # Step 2: Close gripper
        # ==========================================
        logger.info("\n=== Step 2: Closing gripper ===")
        
        # Use action interface to close gripper (simpler and more reliable)
        gripper_goal = GripperCommand.Goal()
        gripper_goal.command.position = 0.0  # Closed position (0.0 = closed, 0.04 = open)
        gripper_goal.command.max_effort = 50.0
        
        logger.info("Sending gripper close command...")
        send_goal_future = gripper_action_client.send_goal_async(gripper_goal)
        rclpy.spin_until_future_complete(node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if goal_handle.accepted:
            logger.info("Gripper goal accepted, waiting for result...")
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)
            result = result_future.result().result
            logger.info(f"✓ Gripper closed (position: {result.position})")
        else:
            logger.warn("Gripper action goal rejected, trying MoveIt planning...")
            # Fallback: Use MoveItPy to set gripper to closed state
            gripper_planner.set_start_state_to_current_state()
            
            robot_model = rtop_moveit.get_robot_model()
            gripper_state = RobotState(robot_model)
            
            # Set gripper joints to closed position
            # Based on SRDF: right_close has right_left_finger=0.037524, right_right_finger=-0.037524
            gripper_state.set_joint_positions("right_left_finger", [0.037524])
            gripper_state.set_joint_positions("right_right_finger", [-0.037524])
            
            gripper_planner.set_goal_state(robot_state=gripper_state)
            
            gripper_plan_result = gripper_planner.plan()
            if gripper_plan_result:
                gripper_trajectory = gripper_plan_result.trajectory
                rtop_moveit.execute(gripper_planning_group, gripper_trajectory, blocking=True)
                logger.info("✓ Gripper closed via MoveIt")
            else:
                logger.error("Failed to close gripper!")

        time.sleep(1.0)

        # ==========================================
        # Step 3: Move to second cartesian pose
        # ==========================================
        logger.info("\n=== Step 3: Moving to second cartesian pose ===")
        arm_planner.set_start_state_to_current_state()

        # Second pose: different position but still reachable by right arm
        # Keep y < 0 (right side) and reasonable height
        pose_goal_2 = PoseStamped()
        pose_goal_2.header.frame_id = base_frame
        pose_goal_2.pose.position.x = 0.4   # Further forward than first pose
        pose_goal_2.pose.position.y = 0.0   # More to the right (negative y for right arm)
        pose_goal_2.pose.position.z = 1.5   # Similar height to first pose (reachable)
        
        # Use same orientation as first pose (we know it works)
        pose_goal_2.pose.orientation.x = 0.0
        pose_goal_2.pose.orientation.y = 0.707
        pose_goal_2.pose.orientation.z = 0.0
        pose_goal_2.pose.orientation.w = 0.707


        logger.info(f"Setting goal pose in frame '{base_frame}':")
        logger.info(f"  Position: x={pose_goal_2.pose.position.x}, y={pose_goal_2.pose.position.y}, z={pose_goal_2.pose.position.z}")
        logger.info(f"  Orientation: w={pose_goal_2.pose.orientation.w}, x={pose_goal_2.pose.orientation.x}, y={pose_goal_2.pose.orientation.y}, z={pose_goal_2.pose.orientation.z}")
        logger.info(f"  Target link: {end_effector_link}")

        arm_planner.set_goal_state(pose_stamped_msg=pose_goal_2, pose_link=end_effector_link)

        logger.info("Planning to goal pose...")
        plan_result = arm_planner.plan()
        if not plan_result:
            logger.error("Failed to plan to second pose!")
            logger.error("Possible reasons:")
            logger.error("  1. Goal pose is outside robot workspace")
            logger.error("  2. Goal pose is in collision")
            logger.error("  3. Goal orientation is not reachable")
            logger.error("  4. Current robot state is invalid")
            logger.error("")
            logger.error("Try adjusting the goal pose coordinates or orientation")
            return 1

        robot_trajectory = plan_result.trajectory
        logger.info("✓ Plan found, executing...")
        rtop_moveit.execute(arm_planning_group, robot_trajectory, blocking=True)
        logger.info("✓ Reached second pose")

        logger.info("\n=== Demo completed successfully! ===")

    except Exception as e:
        logger.error(f"✗ Error: {str(e)}")
        import traceback as tb
        tb.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main())

