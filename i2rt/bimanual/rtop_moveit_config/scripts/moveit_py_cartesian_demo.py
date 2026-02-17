#!/usr/bin/env python3

"""
MoveIt2 Python API script for RTOP bimanual robot.
Sequence:
1. Move right arm to pose_goal_1, open right gripper, wait 2 sec, close right gripper
2. Move left arm to pose_goal_2, open left gripper, wait 2 sec, close left gripper

Usage:
    1. Start the simulation and MoveIt:
       ros2 launch rtop_moveit_config demo.launch.py
    
    2. Run this script:
       python3 src/i2rt/bimanual/rtop_moveit_config/scripts/moveit_py_cartesian_demo.py
       
       Optional: Add --wait-for-key to wait for Enter key press before executing sequence:
       python3 src/i2rt/bimanual/rtop_moveit_config/scripts/moveit_py_cartesian_demo.py --wait-for-key

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
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import os
from geometry_msgs.msg import PoseStamped
import traceback
import time
import yaml
import argparse


def move_gripper(rtop_moveit, gripper_planner, position, side, logger):
    """
    Move gripper to a target position using MoveIt planning.
    position: 'open' or 'close'
    side: 'left' or 'right'
    Returns True on success, False on failure.
    Uses SRDF named targets: 'left_open'/'left_close' or 'right_open'/'right_close'
    """
    gripper_planner.set_start_state_to_current_state()

    # Use SRDF named states directly — avoids RobotState API issues
    config_name = f"{side}_{position}"  # e.g., "right_open" or "left_close"
    controller_name = f"{side}_gripper_controller"
    logger.info(f"Setting {side} gripper goal to SRDF state: '{config_name}'")
    gripper_planner.set_goal_state(configuration_name=config_name)

    plan_result = gripper_planner.plan()
    if plan_result:
        gripper_trajectory = plan_result.trajectory
        rtop_moveit.execute(gripper_trajectory, controllers=[controller_name])
        logger.info(f"✓ {side.capitalize()} gripper {position}ed via MoveIt")
        return True
    else:
        logger.error(f"Failed to {position} {side} gripper!")
        return False


def execute_sequence(rtop_moveit, right_arm_planner, right_gripper_planner,
                     left_arm_planner, left_gripper_planner,
                     right_end_effector_link, left_end_effector_link,
                     base_frame, logger, node):
    """
    Execute the complete sequence:
    1. Move right arm to pose_goal_1
    2. Open right gripper, wait 2 sec, close right gripper
    3. Move left arm to pose_goal_2
    4. Open left gripper, wait 2 sec, close left gripper
    Returns True on success, False on failure.
    """
    try:
        # ==========================================
        # Step 1: Move right arm to pose_goal_1
        # ==========================================
        logger.info("\n=== Step 1: Moving right arm to pose_goal_1 ===")
        
        right_arm_planner.set_start_state_to_current_state()

        # Pose for right arm (negative y for right side)
        pose_goal_1 = PoseStamped()
        pose_goal_1.header.frame_id = base_frame
        pose_goal_1.pose.position.x = 0.0   # Forward from base
        pose_goal_1.pose.position.y = -0.3   # Right side (negative y)
        pose_goal_1.pose.position.z = 1.5   # Height above base
        pose_goal_1.pose.orientation.x = 0.0
        pose_goal_1.pose.orientation.y = 0.0
        pose_goal_1.pose.orientation.z = 0.706
        pose_goal_1.pose.orientation.w = 0.706

        logger.info(f"Setting right arm goal pose in frame '{base_frame}':")
        logger.info(f"  Position: x={pose_goal_1.pose.position.x}, y={pose_goal_1.pose.position.y}, z={pose_goal_1.pose.position.z}")
        logger.info(f"  Orientation: w={pose_goal_1.pose.orientation.w}, x={pose_goal_1.pose.orientation.x}, y={pose_goal_1.pose.orientation.y}, z={pose_goal_1.pose.orientation.z}")
        logger.info(f"  Target link: {right_end_effector_link}")

        right_arm_planner.set_goal_state(pose_stamped_msg=pose_goal_1, pose_link=right_end_effector_link)
        
        logger.info("Planning to goal pose...")
        plan_result = right_arm_planner.plan()
        if not plan_result:
            logger.error("Failed to plan right arm to pose_goal_1!")
            logger.error("Possible reasons:")
            logger.error("  1. Goal pose is outside robot workspace")
            logger.error("  2. Goal pose is in collision")
            logger.error("  3. Goal orientation is not reachable")
            logger.error("  4. Current robot state is invalid")
            return False

        robot_trajectory = plan_result.trajectory
        logger.info("✓ Plan found, executing...")
        rtop_moveit.execute(robot_trajectory, controllers=["right_arm_controller"])
        logger.info("✓ Right arm reached pose_goal_1")

        time.sleep(1.0)

        # ==========================================
        # Step 2: Open right gripper, wait 2 sec, close
        # ==========================================
        logger.info("\n=== Step 2: Opening right gripper ===")
        if not move_gripper(rtop_moveit, right_gripper_planner, "open", "right", logger):
            return False
        
        logger.info("Waiting 2 seconds...")
        time.sleep(2.0)
        
        logger.info("\n=== Step 2b: Closing right gripper ===")
        if not move_gripper(rtop_moveit, right_gripper_planner, "close", "right", logger):
            return False

        time.sleep(1.0)

        # ==========================================
        # Step 3: Move left arm to pose_goal_2
        # ==========================================
        logger.info("\n=== Step 3: Moving left arm to pose_goal_2 ===")
        left_arm_planner.set_start_state_to_current_state()

        # Pose for left arm (positive y for left side)
        pose_goal_2 = PoseStamped()
        pose_goal_2.header.frame_id = base_frame
        pose_goal_2.pose.position.x = 0.63   # Forward from base
        pose_goal_2.pose.position.y = 0.3    # Left side (positive y)
        pose_goal_2.pose.position.z = 1.0    # Height above base
        pose_goal_2.pose.orientation.x = 0.5
        pose_goal_2.pose.orientation.y = 0.5
        pose_goal_2.pose.orientation.z = 0.5
        pose_goal_2.pose.orientation.w = 0.5

        logger.info(f"Setting left arm goal pose in frame '{base_frame}':")
        logger.info(f"  Position: x={pose_goal_2.pose.position.x}, y={pose_goal_2.pose.position.y}, z={pose_goal_2.pose.position.z}")
        logger.info(f"  Orientation: w={pose_goal_2.pose.orientation.w}, x={pose_goal_2.pose.orientation.x}, y={pose_goal_2.pose.orientation.y}, z={pose_goal_2.pose.orientation.z}")
        logger.info(f"  Target link: {left_end_effector_link}")

        left_arm_planner.set_goal_state(pose_stamped_msg=pose_goal_2, pose_link=left_end_effector_link)

        logger.info("Planning to goal pose...")
        plan_result = left_arm_planner.plan()
        if not plan_result:
            logger.error("Failed to plan left arm to pose_goal_2!")
            logger.error("Possible reasons:")
            logger.error("  1. Goal pose is outside robot workspace")
            logger.error("  2. Goal pose is in collision")
            logger.error("  3. Goal orientation is not reachable")
            logger.error("  4. Current robot state is invalid")
            return False

        robot_trajectory = plan_result.trajectory
        logger.info("✓ Plan found, executing...")
        rtop_moveit.execute(robot_trajectory, controllers=["left_arm_controller"])
        logger.info("✓ Left arm reached pose_goal_2")

        time.sleep(1.0)

        # ==========================================
        # Step 4: Open left gripper, wait 2 sec, close
        # ==========================================
        logger.info("\n=== Step 4: Opening left gripper ===")
        if not move_gripper(rtop_moveit, left_gripper_planner, "open", "left", logger):
            return False
        
        logger.info("Waiting 2 seconds...")
        time.sleep(2.0)
        
        logger.info("\n=== Step 4b: Closing left gripper ===")
        if not move_gripper(rtop_moveit, left_gripper_planner, "close", "left", logger):
            return False

        time.sleep(1.0)

        return True
        
    except Exception as e:
        logger.error(f"Error during sequence execution: {str(e)}")
        return False


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='MoveIt2 Python API Cartesian Demo for RTOP')
    parser.add_argument('--wait-for-key', action='store_true',
                        help='Wait for Enter key press before executing the sequence (useful to wait for library loading)')
    args = parser.parse_args()
    
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
        right_arm_planning_group = "right_arm"
        right_gripper_planning_group = "right_gripper"
        left_arm_planning_group = "left_arm"
        left_gripper_planning_group = "left_gripper"
        right_end_effector_link = "right_link_6"
        left_end_effector_link = "left_link_6"
        base_frame = "base"  # Base frame for poses

        logger.info(f"Initializing MoveItPy for bimanual planning groups")

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
        right_arm_planner = rtop_moveit.get_planning_component(right_arm_planning_group)
        right_gripper_planner = rtop_moveit.get_planning_component(right_gripper_planning_group)
        left_arm_planner = rtop_moveit.get_planning_component(left_arm_planning_group)
        left_gripper_planner = rtop_moveit.get_planning_component(left_gripper_planning_group)
        logger.info("✓ MoveItPy initialized for both arms")

        # Main loop: execute sequence repeatedly
        sequence_count = 0
        while True:
            sequence_count += 1
            
            # Wait for key press if requested (before each sequence)
            if args.wait_for_key:
                if sequence_count == 1:
                    logger.info("\n" + "="*60)
                    logger.info("Initialization complete! All libraries loaded.")
                    logger.info("Press ENTER to start the sequence...")
                    logger.info("="*60)
                else:
                    logger.info("\n" + "="*60)
                    logger.info(f"Sequence #{sequence_count - 1} completed!")
                    logger.info("Press ENTER to run the sequence again (or Ctrl+C to exit)...")
                    logger.info("="*60)
                try:
                    input()
                except (EOFError, KeyboardInterrupt):
                    logger.info("\nExiting...")
                    break
                logger.info("Starting sequence...\n")

            # Execute the sequence
            success = execute_sequence(
                rtop_moveit, right_arm_planner, right_gripper_planner,
                left_arm_planner, left_gripper_planner,
                right_end_effector_link, left_end_effector_link,
                base_frame, logger, node
            )
            
            if success:
                logger.info("\n=== Sequence completed successfully! ===")
            else:
                logger.error("\n=== Sequence failed! ===")
                if args.wait_for_key:
                    logger.info("Press ENTER to try again (or Ctrl+C to exit)...")
                    try:
                        input()
                    except (EOFError, KeyboardInterrupt):
                        logger.info("\nExiting...")
                        break
                else:
                    # If not waiting for key, ask if user wants to retry
                    print("Would you like to try again? (y/n): ", end='')
                    try:
                        response = input().strip().lower()
                        if response != 'y':
                            break
                    except (EOFError, KeyboardInterrupt):
                        logger.info("\nExiting...")
                        break
            
            # If not using wait-for-key, ask if user wants to run again
            if not args.wait_for_key:
                print("\nWould you like to run the sequence again? (y/n): ", end='')
                try:
                    response = input().strip().lower()
                    if response != 'y':
                        break
                except (EOFError, KeyboardInterrupt):
                    logger.info("\nExiting...")
                    break

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
