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
    
    # Paths
    urdf_file_path = PathJoinSubstitution([pkg_share, "urdf", "rtop.urdf.xacro"])
    
    # Default empty world path from ros_gz_sim
    default_world_path = os.path.join(ros_gz_sim_dir, "worlds", "empty.sdf")
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world_path,
        description="Full path to world SDF file (default: empty world)"
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
    
    robot_yaw_arg = DeclareLaunchArgument(
        "robot_yaw",
        default_value="0.0",
        description="Robot spawn yaw angle"
    )
    
    # Gazebo Sim Launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", LaunchConfiguration("world")],
            "on_exit_shutdown": "true"
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_file_path]),
                value_type=str
            )
        }]
    )
    
    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=[
            "-name", "rtop",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("robot_x"),
            "-y", LaunchConfiguration("robot_y"),
            "-z", "0.2",
            "-Y", LaunchConfiguration("robot_yaw")
        ]
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
        robot_yaw_arg,
        gazebo_sim,
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_joint_states,
        bridge_clock,
    ])
