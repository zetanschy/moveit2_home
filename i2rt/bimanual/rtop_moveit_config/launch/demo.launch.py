from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    return LaunchDescription([
        use_sim_time_arg,
        set_sim_time,
        *demo_launch.entities,
    ])
