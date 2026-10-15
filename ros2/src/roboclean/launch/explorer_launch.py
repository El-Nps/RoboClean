from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    persistent_map_dir = LaunchConfiguration('persistent_map_dir')
    persistent_map_name = LaunchConfiguration('persistent_map_name')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_persistent_map_dir_cmd = DeclareLaunchArgument(
        'persistent_map_dir',
        default_value='/opt/tmp/map',
        description='Directory for persistent map files'
    )

    declare_persistent_map_name_cmd = DeclareLaunchArgument(
        'persistent_map_name',
        default_value='roboclean_map',
        description='Base name for persistent map files'
    )

    explorer_node = Node(
        package='roboclean',
        executable='explorer',
        name='explorer_node',
        output='screen', # Log to screen
        parameters=[{
            'use_sim_time': use_sim_time,
            'persistent_map_dir': persistent_map_dir,
            'persistent_map_name': persistent_map_name,
        }]
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_persistent_map_dir_cmd,
        declare_persistent_map_name_cmd,
        explorer_node
    ])
