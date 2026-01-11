#!/usr/bin/env python3
"""
Gazebo Simulation Launch File for RTOP Robot

This launch file starts:
1. Gazebo Sim with a custom world
2. Robot State Publisher - publishes the robot's URDF
3. Spawns the robot into Gazebo
4. ROS-Gazebo Bridge for communication

Usage:
    ros2 launch rtop_description gazebo.launch.py
    
Optional arguments:
    world:=<world_file>           - Specify world file (default: empty.sdf)
    robot_x:=<x_position>         - X spawn position (default: 0.0)
    robot_y:=<y_position>         - Y spawn position (default: 0.0)
    robot_z:=<z_position>         - Z spawn position (default: 0.0)
    robot_yaw:=<yaw_angle>        - Yaw spawn angle (default: 0.0)
"""

import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package name
    package_name = "rtop_description"
    
    # Get package directories
    pkg_share = FindPackageShare(package=package_name)
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    # Get package share directories for mesh resolution
    rtop_description_share = get_package_share_directory("rtop_description")
    yam_arm_description_share = get_package_share_directory("yam_arm_description")
    
    # Set up Gazebo resource path so it can find model:// URIs
    # When URDF is converted to SDF, package:// URIs become model:// URIs
    # Gazebo looks for model://package_name/path in directories that contain
    # the package directories. The structure is: install/package_name/share/package_name/
    # So we need to point to the share/ directory (parent of package share dir)
    # This allows Gazebo to find model://package_name/path at share/package_name/path
    rtop_description_share_parent = os.path.dirname(rtop_description_share)  # Points to share/
    yam_arm_description_share_parent = os.path.dirname(yam_arm_description_share)  # Points to share/
    
    # Also get the install directory to cover all packages
    install_dir = os.path.dirname(rtop_description_share_parent)  # Points to install/
    
    # Build resource path - point to share directories so Gazebo can resolve model:// URIs
    # Format: model://package_name/path -> share/package_name/path
    gz_resource_path = f"{rtop_description_share_parent}:{yam_arm_description_share_parent}:{install_dir}"
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_resource_path = f"{os.environ['GZ_SIM_RESOURCE_PATH']}:{gz_resource_path}"
    
    # Also set IGN_GAZEBO_RESOURCE_PATH for compatibility
    ign_resource_path = gz_resource_path
    
    # Set environment variables for Gazebo processes
    set_gz_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        gz_resource_path
    )
    set_ign_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        ign_resource_path
    )
    
    # Paths
    urdf_file_path = PathJoinSubstitution([pkg_share, "urdf", "rtop.urdf.xacro"])
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="",
        description="Full path to world SDF file (empty for default empty world)"
    )
    
    robot_x_arg = DeclareLaunchArgument(
        "robot_x",
        default_value="0.0",
        description="Robot spawn X position"
    )
    
    robot_y_arg = DeclareLaunchArgument(
        "robot_y",
        default_value="0.0",
        description="Robot spawn Y position"
    )
    
    robot_z_arg = DeclareLaunchArgument(
        "robot_z",
        default_value="0.0",
        description="Robot spawn Z position"
    )
    
    robot_yaw_arg = DeclareLaunchArgument(
        "robot_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle"
    )
    
    # Gazebo Sim Launch
    # Pass empty list for gz_args to use default empty world
    # If you want to specify a world file, pass it as: world:=/path/to/world.sdf
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [],
            "on_exit_shutdown": "true"
        }.items()
    )
    
    # Robot State Publisher
    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_file_path]),
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description_content
        }]
    )
    
    # Spawn Robot in Gazebo
    # NOTE: There is a known issue with Gazebo Fortress where URDF->SDF conversion
    # can create duplicate joints when using -topic option. This is a Gazebo bug.
    # Workaround: The robot works fine in Isaac Sim because it handles URDF differently.
    # For Gazebo, you may need to manually convert URDF to SDF first and use -file option.
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_rtop",
        output="screen",
        arguments=[
            "-name", "rtop",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("robot_x"),
            "-y", LaunchConfiguration("robot_y"),
            "-z", LaunchConfiguration("robot_z"),
            "-Y", LaunchConfiguration("robot_yaw")
        ]
    )
    
    # Delay spawn to ensure robot_state_publisher has published and Gazebo is ready
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot_node]
    )
    
    # ROS-Gazebo Bridge for joint_states (bidirectional)
    bridge_joint_states = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_joint_states",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        output="screen"
    )
    
    # ROS-Gazebo Bridge for clock
    bridge_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bridge_clock",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"
        ],
        output="screen"
    )
    
    # Create and return launch description
    return LaunchDescription([
        world_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_yaw_arg,
        set_gz_resource_path,  # Set before Gazebo launches
        set_ign_resource_path,  # Also set IGN path for compatibility
        gazebo_sim,
        robot_state_publisher_node,
        delayed_spawn,
        bridge_joint_states,
        bridge_clock,
    ])
