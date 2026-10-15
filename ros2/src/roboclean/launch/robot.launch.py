import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo, OpaqueFunction, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _has_valid_saved_map(map_yaml, fallback_map_image):
    if not (os.path.exists(map_yaml) and os.path.getsize(map_yaml) > 0):
        return False

    image_path = fallback_map_image
    try:
        with open(map_yaml, 'r', encoding='utf-8') as yaml_file:
            for raw_line in yaml_file:
                line = raw_line.strip()
                if line.startswith('image:'):
                    image_value = line.split(':', 1)[1].strip().strip('"').strip("'")
                    if image_value:
                        if os.path.isabs(image_value):
                            image_path = image_value
                        else:
                            image_path = os.path.join(os.path.dirname(map_yaml), image_value)
                    break
    except OSError:
        return False

    return os.path.exists(image_path) and os.path.getsize(image_path) > 0


def _load_saved_initial_pose(map_dir, map_name, fallback_x, fallback_y, fallback_yaw):
    pose_file = os.path.join(map_dir, f'{map_name}.initial_pose.json')
    if not os.path.exists(pose_file):
        return fallback_x, fallback_y, fallback_yaw

    try:
        with open(pose_file, 'r', encoding='utf-8') as f:
            payload = json.load(f)
        pose_x = float(payload.get('x', fallback_x))
        pose_y = float(payload.get('y', fallback_y))
        pose_yaw = float(payload.get('yaw', fallback_yaw))
        return pose_x, pose_y, pose_yaw
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        return fallback_x, fallback_y, fallback_yaw


def _create_nav2_bringup(context, pkg_share, nav2_pkg_share, nav2_params_file):
    map_mode = context.launch_configurations.get('map_mode', 'auto').lower()
    map_dir = context.launch_configurations.get('map_dir', '/opt/tmp/map')
    map_name = context.launch_configurations.get('map_name', 'roboclean_map')
    map_yaml = os.path.join(map_dir, f'{map_name}.yaml')
    map_pgm = os.path.join(map_dir, f'{map_name}.pgm')
    posegraph_file = os.path.join(map_dir, f'{map_name}.posegraph')
    has_saved_map = _has_valid_saved_map(map_yaml, map_pgm)
    has_posegraph = os.path.exists(posegraph_file) and os.path.getsize(posegraph_file) > 0

    if map_mode not in ('auto', 'mapping', 'lifelong', 'localization'):
        raise RuntimeError("map_mode must be one of: 'auto', 'mapping', 'lifelong', 'localization'")

    if map_mode == 'auto':
        resolved_mode = 'lifelong' if has_saved_map and has_posegraph else 'mapping'
    else:
        resolved_mode = map_mode

    if resolved_mode == 'lifelong' and not (has_saved_map and has_posegraph):
        raise RuntimeError(
            'map_mode=lifelong requires existing map files '
            f'({map_yaml}, {map_pgm}) and posegraph file ({posegraph_file})'
        )

    try:
        fallback_initial_x = float(context.launch_configurations.get('initial_x', '0.0'))
        fallback_initial_y = float(context.launch_configurations.get('initial_y', '0.0'))
        fallback_initial_yaw = float(context.launch_configurations.get('initial_yaw', '0.0'))
    except ValueError as exc:
        raise RuntimeError(f'Invalid fallback initial pose values: {exc}')

    slam_enabled = resolved_mode in ('mapping', 'lifelong')

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites={
            'map_file_name': posegraph_file,
        },
        convert_types=True,
    )

    launch_arguments = {
        'slam': 'True' if slam_enabled else 'False',
        'params_file': configured_nav2_params,
        'use_sim_time': 'False',
        'autostart': 'True',
        'log_level': 'warn'
    }
    if resolved_mode in ('lifelong', 'localization'):
        launch_arguments['map'] = map_yaml

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_pkg_share, '/launch', '/bringup_launch.py'
        ]),
        launch_arguments=launch_arguments.items()
    )

    actions = [
        LogInfo(msg=f'[roboclean] map_mode={resolved_mode} (yaml: {map_yaml}, pgm: {map_pgm})'),
        nav2_bringup_launch,
    ]

    if not slam_enabled:
        initial_x, initial_y, initial_yaw = _load_saved_initial_pose(
            map_dir,
            map_name,
            fallback_initial_x,
            fallback_initial_y,
            fallback_initial_yaw,
        )
        actions.append(
            Node(
                package='roboclean',
                executable='initial_pose_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': False,
                    'initial_x': initial_x,
                    'initial_y': initial_y,
                    'initial_yaw': initial_yaw,
                    'frame_id': 'map',
                    'publish_period': 0.5,
                    'publish_count': 20,
                }],
            )
        )

    return actions


def generate_launch_description():
    pkg_share = FindPackageShare("roboclean")
    nav2_pkg_share = FindPackageShare("nav2_bringup")

    # Robot Description (URDF)
    robot_desc_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    ])
    robot_description = {"robot_description": robot_desc_content}

    # State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Base Controller
    base_controller_node = Node(
        package="roboclean",
        executable="base_controller",
        output="screen",
        parameters=[{
            "serial_port": "/dev/sensors/arduino",
            "baud_rate": 115200,
            "wheel_radius": 0.0375,
            "wheel_separation": 0.2575,
            "pulses_per_rotation": 519,
            "max_rpm": 60.0,
            "base_frame": "base_link"
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # RPLidar
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/sensors/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }],
        respawn=True,
        respawn_delay=5.0,
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Navigation Stack (Nav2)
    nav2_params_file = PathJoinSubstitution([
        pkg_share, "params", "nav2_params.yaml"
    ])
    
    nav2_bringup_launch = OpaqueFunction(
        function=_create_nav2_bringup,
        args=[pkg_share, nav2_pkg_share, nav2_params_file],
    )

    # Block until the first REAL /scan message is published by the Lidar
    wait_for_scan = Node(
        package='roboclean',
        executable='wait_for_scan',
        output='screen'
    )

    # Once the Lidar successfully spins up and publishes the first scan, we 
    # finally trigger the Navigation 2 stack!
    start_nav2_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[nav2_bringup_launch]
        )
    )

    # Watchdog to prevent silent failures (deadlocks) in the Lidar hardware.
    # ROS 2's respawn=True ONLY works if the process actually crashes. If the Lidar
    # motor gets stuck or the USB bus drops silently, the node stays alive but publishes 
    # nothing. This watchdog aggressively kills the node if 5 seconds pass without a scan,
    # forcing `respawn=True` to do its job!
    lidar_watchdog = Node(
        package='roboclean',
        executable='lidar_watchdog',
        output='screen'
    )

    # Web Bridge
    web_bridge_node = Node(
        package='roboclean',
        executable='web_bridge',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Barrier Publisher
    barrier_publisher_node = Node(
        package='roboclean',
        executable='barrier_publisher',
        output='screen',
        parameters=[{
            'map_dir': LaunchConfiguration('map_dir'),
            'map_basename': LaunchConfiguration('map_name')
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # System Monitor
    system_monitor_node = Node(
        package='roboclean',
        executable='system_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_mode',
            default_value='auto',
            description="Map source mode: 'auto' | 'mapping' | 'lifelong' | 'localization'"
        ),
        DeclareLaunchArgument(
            'map_dir',
            default_value='/opt/tmp/map',
            description='Directory for persistent map files'
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='roboclean_map',
            description='Base name for persistent map files (<name>.yaml/.pgm)'
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Fallback initial pose x (used if no saved initial pose metadata exists)'
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Fallback initial pose y (used if no saved initial pose metadata exists)'
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Fallback initial yaw in radians (used if no saved initial pose metadata exists)'
        ),
        robot_state_pub_node,
        base_controller_node,
        rplidar_node,        # Starts immediately, respawns if crashed
        lidar_watchdog,      # Will kill the Lidar process if it silently freezes
        wait_for_scan,       # Blocks until /scan has actual data
        start_nav2_event,    # Triggers Nav2 once wait_for_scan exits
        web_bridge_node,
        barrier_publisher_node,
        system_monitor_node
    ])
