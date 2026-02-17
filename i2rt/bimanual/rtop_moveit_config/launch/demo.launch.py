from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import SetParameter, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo/Isaac) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # SetParameter applies use_sim_time to all nodes launched after it
    set_sim_time = SetParameter(name="use_sim_time", value=use_sim_time)

    moveit_config = MoveItConfigsBuilder("rtop_calib", package_name="rtop_moveit_config").to_moveit_configs()
    launch_package_path = moveit_config.package_path

    # Process URDF explicitly to ensure robot_description is available
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare("rtop_moveit_config"),
        "config",
        "rtop_calib.urdf.xacro"
    ])
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file_path]),
        value_type=str
    )

    # --- Bridge node (Isaac Sim) ---
    def create_bridge_node(context):
        use_sim_time_val = context.launch_configurations.get('use_sim_time', 'false')
        return [
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'source /opt/ros/jazzy/setup.bash && '
                    f'source /workspace/robot/ros2/install/setup.bash 2>/dev/null || true && '
                    f'source /workspace/install/setup.bash 2>/dev/null || true && '
                    f'ros2 run rtop moveit2_isaac_bridge_node --ros-args -p use_sim_time:={use_sim_time_val}'
                ],
                output='screen',
                name='moveit2_isaac_bridge_node',
            )
        ]

    bridge_node_action = OpaqueFunction(function=create_bridge_node)

    # --- Robot State Publisher (single instance) ---
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    # --- Controller Manager (single instance) ---
    ros2_controllers_path = PathJoinSubstitution([
        FindPackageShare("rtop_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    ])

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            ros2_controllers_path,
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    # --- Controller Spawners ---
    # Spawn all controllers from moveit_controllers.yaml + joint_state_broadcaster
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    spawners = []
    for controller in controller_names + ["joint_state_broadcaster"]:
        spawners.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # --- MoveIt2 components (from individual launch files, NO generate_demo_launch) ---
    # This avoids the duplicate ros2_control_node that generate_demo_launch creates

    # Optional: static virtual joint TFs
    static_virtual_joint_tfs_launch = launch_package_path / "launch/static_virtual_joint_tfs.launch.py"

    # move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/move_group.launch.py")
        ),
    )

    # rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(launch_package_path / "launch/moveit_rviz.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # --- Assemble launch ---
    ld = LaunchDescription()

    # Declare args
    ld.add_action(use_sim_time_arg)
    ld.add_action(set_sim_time)
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # Bridge node (launches first)
    ld.add_action(bridge_node_action)

    # Static virtual joint TFs (if the launch file exists)
    if static_virtual_joint_tfs_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(static_virtual_joint_tfs_launch)),
            )
        )

    # Robot state publisher + controller manager (with delay)
    delayed_core = TimerAction(
        period=2.0,
        actions=[
            robot_state_publisher_node,
            controller_manager_node,
        ]
    )
    ld.add_action(delayed_core)

    # Spawners + MoveIt2 (after controller_manager is ready)
    delayed_moveit = TimerAction(
        period=4.0,
        actions=[
            *spawners,
            move_group_launch,
            rviz_launch,
        ]
    )
    ld.add_action(delayed_moveit)

    return ld
