from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os


def generate_launch_description():
    # Default to loading params.yaml from automated_planner if it exists
    default_params_file = ''
    try:
        planner_share = get_package_share_directory('okmr_automated_planner')
        default_params_file = os.path.join(
            planner_share, 'state_machine_configs', 'dev', 'params.yaml'
        )
    except (PackageNotFoundError, FileNotFoundError):
        pass
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to params.yaml file to load (optional)'
    )
    
    # Note: params_file is a command line argument, not a ROS parameter
    tune_gui_node = Node(
        package='tuneGUI',
        executable='tune_gui',
        name='tune_gui_node',
        output='screen',
        arguments=[LaunchConfiguration('params_file')] if default_params_file else []
    )
    
    return LaunchDescription([
        params_file_arg,
        tune_gui_node,
    ])