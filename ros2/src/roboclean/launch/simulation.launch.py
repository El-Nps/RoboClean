import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
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


def _create_nav2_bringup(context, nav2_bringup_pkg, nav2_params_file, use_sim_time):
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
        spawn_x = float(context.launch_configurations.get('x_pose', '0.0'))
        spawn_y = float(context.launch_configurations.get('y_pose', '0.0'))
        spawn_yaw = float(context.launch_configurations.get('yaw', '0.0'))
    except ValueError as exc:
        raise RuntimeError(f'Invalid spawn pose values for localization startup: {exc}')

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
        'use_sim_time': use_sim_time,
        'slam': 'True' if slam_enabled else 'False',
        'params_file': configured_nav2_params,
        'autostart': 'True',
        'log_level': 'warn'
    }
    if resolved_mode in ('lifelong', 'localization'):
        launch_arguments['map'] = map_yaml

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_pkg, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments=launch_arguments.items()
    )

    actions = [
        LogInfo(msg=f'[roboclean] map_mode={resolved_mode} (yaml: {map_yaml}, pgm: {map_pgm})'),
        nav2_bringup,
    ]

    if not slam_enabled:
        initial_x, initial_y, initial_yaw = _load_saved_initial_pose(
            map_dir,
            map_name,
            spawn_x,
            spawn_y,
            spawn_yaw,
        )
        actions.append(
            Node(
                package='roboclean',
                executable='initial_pose_publisher',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
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
    nav2_bringup_pkg = FindPackageShare("nav2_bringup")
    ros_gz_sim_pkg = FindPackageShare("ros_gz_sim")

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    default_world_path = PathJoinSubstitution([pkg_share, 'worlds', 'floorplan.sdf'])
    world_path = LaunchConfiguration('world', default=default_world_path)
    
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.05')
    yaw_pose = LaunchConfiguration('yaw', default='0.0')

    # Gazebo Server (Headless)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': ['-r -s -v 4 ', world_path]}.items()
    )
    
    # Robot State Publisher
    robot_desc_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg_share, "urdf", "robot.urdf.xacro"])
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_desc_content, 'use_sim_time': use_sim_time}]
    )

    # Gazebo Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'vacuum_bot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    # Bridge ROS 2 <-> Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/vacuum_bot/pose@geometry_msgs/msg/Pose@gz.msgs.Pose', 
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Nav2 Bringup
    nav2_params_file = PathJoinSubstitution([pkg_share, "params", "nav2_params.yaml"])
    nav2_bringup = OpaqueFunction(
        function=_create_nav2_bringup,
        args=[nav2_bringup_pkg, nav2_params_file, use_sim_time],
    )
    
    # Block until the first /scan message is published by Gazebo bridge
    wait_for_scan = Node(
        package='roboclean',
        executable='wait_for_scan',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_nav2_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_scan,
            on_exit=[nav2_bringup]
        )
    )

    # Web Bridge
    web_bridge = Node(
        package='roboclean',
        executable='web_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Barrier Publisher
    barrier_publisher_node = Node(
        package='roboclean',
        executable='barrier_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_dir': LaunchConfiguration('map_dir'),
            'map_basename': LaunchConfiguration('map_name')
        }],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Static TF (Lidar frame fix)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'laser_frame', 'vacuum_bot/base_footprint/lidar'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # System Monitor
    system_monitor_node = Node(
        package='roboclean',
        executable='system_monitor',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world_path),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('z_pose', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
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
        
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        wait_for_scan,
        start_nav2_event,
        web_bridge,
        barrier_publisher_node,
        lidar_tf,
        system_monitor_node,
    ])
