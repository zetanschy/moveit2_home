from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


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
    demo_launch = generate_demo_launch(moveit_config)
    
    # Bridge node to add mimic joints to joint commands for Isaac Sim
    # The rtop package is in /workspace/robot/ros2, so we need to source that workspace
    # Use ExecuteProcess with a function to resolve use_sim_time at runtime
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
    
    return LaunchDescription([
        use_sim_time_arg,
        set_sim_time,
        bridge_node_action,  # Launch bridge node first
        *demo_launch.entities,
    ])
