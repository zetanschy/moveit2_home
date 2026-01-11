from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for Isaac Sim URDF importer.
    Publishes robot_description on /robot_description topic.
    """
    # Get package share directory
    rtop_description_share = get_package_share_directory('rtop_description')
    yam_arm_description_share = get_package_share_directory('yam_arm_description')
    
    # Set ROS_PACKAGE_PATH for Isaac Sim to resolve package:// URIs
    # Isaac Sim needs ROS_PACKAGE_PATH to find ROS packages
    workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(rtop_description_share))))
    src_dir = os.path.join(workspace_dir, 'src')
    
    # Build ROS_PACKAGE_PATH with source directories
    ros_package_path = f"{os.path.join(src_dir, 'i2rt', 'bimanual')}:{os.path.join(src_dir, 'i2rt', 'single')}"
    if 'ROS_PACKAGE_PATH' in os.environ:
        ros_package_path = f"{os.environ['ROS_PACKAGE_PATH']}:{ros_package_path}"
    
    set_ros_package_path = SetEnvironmentVariable(
        'ROS_PACKAGE_PATH',
        ros_package_path
    )
    
    # URDF file path
    urdf_file = os.path.join(rtop_description_share, 'urdf', 'rtop.urdf.xacro')
    
    # Process xacro to get URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot state publisher node - publishes robot_description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (set to true for Isaac Sim)'
    )
    
    return LaunchDescription([
        set_ros_package_path,  # Set ROS_PACKAGE_PATH before publishing URDF
        use_sim_time_arg,
        robot_state_publisher_node,
    ])

