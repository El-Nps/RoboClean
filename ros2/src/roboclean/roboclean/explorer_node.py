import json
import math
import os
import struct

import numpy as np
from scipy.ndimage import convolve, distance_transform_edt, label

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from tf2_ros import Buffer, LookupException, TimeoutException, TransformException, TransformListener

from .utils.metrics import ExplorationMetricsTracker


def save_occupancy_grid_as_pgm(occupancy_grid, filename):
    """Convert ROS OccupancyGrid to PGM grayscale image"""
    width = occupancy_grid.info.width
    height = occupancy_grid.info.height
    data = occupancy_grid.data
    
    # Create directory if needed
    os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
    
    # Write PGM header (P5 = binary grayscale)
    with open(filename, "wb") as f:
        f.write(b"P5\n")
        f.write(b"# Map snapshot\n")
        f.write(f"{width} {height}\n".encode())
        f.write(b"255\n")  # Max pixel value
        
        # Convert map data to grayscale pixels
        for value in data:
            if value < 0:       # Unknown (-1)
                pixel = 128     # Gray
            elif value == 0:    # Free space (0)
                pixel = 255     # White
            else:               # Occupied (0-100)
                pixel = int((100 - value) * 255 / 100)  # Black for occupied
            
            f.write(struct.pack('B', pixel))

class ExplorerNode(Node):
    def __init__(self):
        super().__init__('simple_explorer')
        
        # Initialize metrics tracker with logger
        self.metrics = ExplorationMetricsTracker(logger=self.get_logger())

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            map_qos
        )
        
        # Odometry subscription to track distance
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_saver_client = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.posegraph_serializer_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.start_time = self.get_clock().now()
        self.goal_in_progress = False
        self.goal_start_time = None
        self.timer = self.create_timer(2.0, self.explore)
        self.should_exit = False
        self.blacklisted_goals = [] # List of (x, y, timestamp)
        
        # Performance tuning parameters
        self.max_wait_timeout = 60.0
        self.min_goal_duration = 3.0
        self.min_number_frontiers = 1
        self.min_cells_from_wall = 8
        self.blacklist_radius = 0.5
        self.blacklist_duration = 120.0
        
        # Metrics timer
        self.metrics_timer = self.create_timer(10.0, self.record_metrics)
        
        # Map snapshot tracking
        self.last_map_save = self.get_clock().now()
        self.map_save_interval = 15.0  # Save every 15 seconds
        self.map_save_count = 0
        self.exploration_finished = False
        self.map_save_in_progress = False
        self.initial_pose_map = None

        # Persistent map output (overwritten on each completed exploration)
        self.declare_parameter('persistent_map_dir', '/opt/tmp/map')
        self.declare_parameter('persistent_map_name', 'roboclean_map')
        self.persistent_map_dir = self.get_parameter('persistent_map_dir').get_parameter_value().string_value
        self.persistent_map_name = self.get_parameter('persistent_map_name').get_parameter_value().string_value
    
    def odom_callback(self, msg: Odometry):
        self.metrics.record_position(msg)
    
    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        
        # Save map every N seconds
        current_time = self.get_clock().now()
        elapsed = (current_time - self.last_map_save).nanoseconds / 1e9
        if elapsed > self.map_save_interval:
            self.save_map_snapshot()
            self.last_map_save = current_time
    
    def save_map_snapshot(self):
        if self.map_data is None:
            return
        
        # Use nanoseconds for timestamp
        timestamp = int(self.get_clock().now().nanoseconds / 1e6)
        width = self.map_data.info.width
        height = self.map_data.info.height

        # Save metadata as JSON
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        metadata = {
            'timestamp': timestamp,
            'elapsed_time': elapsed_time,
            'coverage_percent': self.metrics.calculate_map_coverage(self.map_data),
            'distance_m': self.metrics.total_distance,
            'goals_sent': self.metrics.goals_sent,
            'goals_completed': self.metrics.goals_completed,
            'width': width,
            'height': height,
            'resolution': float(self.map_data.info.resolution),
        }
        
        filename_base = f"/tmp/map_{self.map_save_count:03d}"
        
        # Save JSON metadata
        with open(f"{filename_base}.json", 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save PGM image
        # save_occupancy_grid_as_pgm(self.map_data, f"{filename_base}.pgm")
        
        self.get_logger().info(
            f"Saved map snapshot #{self.map_save_count}: "
            f"coverage={metadata['coverage_percent']:.1f}%, "
            f"distance={metadata['distance_m']:.2f}m"
        )
        
        self.map_save_count += 1
    
    def record_metrics(self):
        if self.map_data is None:
            return
        
        snapshot = self.metrics.get_snapshot(self.map_data)
        self.get_logger().info(
            f'\n{"━"*40}\n'
            f'METRICS\n'
            f'{"━"*40}\n'
            f'Coverage:       {snapshot.map_coverage:.1f}%\n'
            f'Distance:       {snapshot.total_distance:.2f}m\n'
            f'Frontiers:      {snapshot.num_frontiers}\n'
            f'Goals:          {snapshot.goals_sent}/{snapshot.goals_completed}\n'
            f'Time:           {snapshot.elapsed_time:.1f}s\n'
            f'{"━"*40}\n'
        )

    def explore(self):
        if self.exploration_finished:
            return

        if self.map_save_in_progress:
            return

        if self.map_data is None:
            self.get_logger().info('Waiting for map...')
            return

        if self.goal_in_progress and self.goal_start_time is not None:
            elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
            if elapsed > self.max_wait_timeout:
                self.get_logger().warn(f'Goal timeout ({elapsed:.1f}s). Cancelling.')
                
                # Blacklist current goal pose if available because it's unreachable
                if hasattr(self, 'current_goal_pose'):
                    self.blacklist_goal(
                        self.current_goal_pose.pose.position.x,
                        self.current_goal_pose.pose.position.y
                    )

                self.cancel_current_goal()
                self.goal_in_progress = False
                self.goal_start_time = None
                return

        if self.goal_in_progress:
            return

        # Check transform availability (map -> odom)
        try:
            _ = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, TimeoutException, TransformException):
            self.get_logger().warn('TF map->odom not available.')
            return

        self.capture_initial_pose_if_needed()

        goal = self.find_frontier_goal()

        if goal:
            self.send_navigation_goal(goal)
        else:
            self.get_logger().info('No frontier found. Stopping exploration.')
            self.exploration_finished = True

            # Save metrics when exploration completes
            self.save_map_snapshot()
            self.metrics.save_metrics('/tmp/exploration_metrics.json')
            self.metrics.print_summary()
            self.save_initial_pose_metadata()
            self.save_persistent_map()

            self.get_logger().info('Metrics saved to /tmp/exploration_metrics.json')
            return

    def capture_initial_pose_if_needed(self):
        if self.initial_pose_map is not None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x, y, yaw = pose
        self.initial_pose_map = {
            'x': x,
            'y': y,
            'yaw': yaw,
            'frame_id': 'map'
        }
        self.get_logger().info(
            f"Captured initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )

    def save_initial_pose_metadata(self):
        if self.initial_pose_map is None:
            self.capture_initial_pose_if_needed()

        if self.initial_pose_map is None:
            self.get_logger().warn('No initial pose captured. Skipping initial pose metadata save.')
            return

        os.makedirs(self.persistent_map_dir, exist_ok=True)
        pose_file = os.path.join(
            self.persistent_map_dir,
            f'{self.persistent_map_name}.initial_pose.json'
        )

        try:
            with open(pose_file, 'w', encoding='utf-8') as f:
                json.dump(self.initial_pose_map, f, indent=2)
            self.get_logger().info(f'Saved initial pose metadata to {pose_file}')
        except OSError as exc:
            self.get_logger().error(f'Failed to save initial pose metadata: {exc}')

    def save_persistent_map(self):
        if self.map_data is None:
            self.get_logger().warn('Map not available yet, skipping persistent map save.')
            self.should_exit = True
            return

        map_url = os.path.join(self.persistent_map_dir, self.persistent_map_name)
        os.makedirs(self.persistent_map_dir, exist_ok=True)

        if not self.map_saver_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Map saver service /slam_toolbox/save_map not available.')
            self.should_exit = True
            return

        request = SaveMap.Request()
        request.name.data = map_url

        self.map_save_in_progress = True
        future = self.map_saver_client.call_async(request)
        future.add_done_callback(self._on_persistent_map_saved)

    def _on_persistent_map_saved(self, future):
        self.map_save_in_progress = False

        map_url = os.path.join(self.persistent_map_dir, self.persistent_map_name)
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to save persistent map to {map_url}.yaml/.pgm: {exc}')
            self.should_exit = True
            return

        if response is None:
            self.get_logger().error(f'Failed to save persistent map to {map_url}.yaml/.pgm')
            self.should_exit = True
            return

        result_value = getattr(response, 'result', True)
        if isinstance(result_value, bool):
            success = result_value
        elif isinstance(result_value, int):
            success = (result_value == getattr(response, 'RESULT_SUCCESS', 0))
        else:
            success = True

        if not success:
            self.get_logger().error(f'Map saver reported failure for {map_url}.yaml/.pgm')
            self.should_exit = True
            return

        self.get_logger().info(f'Saved persistent map to {map_url}.yaml/.pgm')
        self.save_persistent_posegraph()

    def save_persistent_posegraph(self):
        posegraph_file = os.path.join(self.persistent_map_dir, f'{self.persistent_map_name}.posegraph')

        if not self.posegraph_serializer_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Serialize service /slam_toolbox/serialize_map not available. Skipping posegraph save.')
            self.should_exit = True
            return

        request = SerializePoseGraph.Request()
        request.filename = posegraph_file

        future = self.posegraph_serializer_client.call_async(request)
        future.add_done_callback(self._on_posegraph_saved)

    def _on_posegraph_saved(self, future):
        posegraph_file = os.path.join(self.persistent_map_dir, f'{self.persistent_map_name}.posegraph')
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Failed to serialize SLAM posegraph to {posegraph_file}: {exc}')
            self.should_exit = True
            return

        if response is None:
            self.get_logger().error(f'Failed to serialize SLAM posegraph to {posegraph_file}')
            self.should_exit = True
            return

        result_value = getattr(response, 'result', True)
        if isinstance(result_value, bool):
            success = result_value
        elif isinstance(result_value, int):
            success = (result_value == getattr(response, 'RESULT_SUCCESS', 0))
        else:
            success = True

        if success:
            self.get_logger().info(f'Serialized SLAM posegraph to {posegraph_file}')
        else:
            self.get_logger().error(f'SLAM posegraph serialization failed for {posegraph_file} (result={result_value})')

        self.should_exit = True

    def get_robot_position(self):
        pose = self.get_robot_pose()
        if pose is None:
            return None
        x, y, _ = pose
        return x, y

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.rotation.z
            w = tf.transform.rotation.w
            yaw = math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)
            self.get_logger().debug(f'Using transform-based position: ({x:.2f}, {y:.2f})')
            return x, y, yaw
        except (LookupException, TimeoutException, TransformException) as e:
            self.get_logger().warn(f'No odometry data and transform lookup failed: {type(e).__name__}')
            return None

    def blacklist_goal(self, x, y):
        """Add a location to the blacklist"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.blacklisted_goals.append((x, y, timestamp))
        self.get_logger().info(f"Blacklisted goal at ({x:.2f}, {y:.2f})")

    def is_goal_blacklisted(self, x, y):
        """Check if a location is close to any blacklisted goal"""
        now = self.get_clock().now().nanoseconds / 1e9
        # Remove expired entries
        self.blacklisted_goals = [
            g for g in self.blacklisted_goals 
            if now - g[2] < self.blacklist_duration
        ]
        
        for bx, by, _ in self.blacklisted_goals:
            distance = np.hypot(x - bx, y - by)
            if distance < self.blacklist_radius:
                return True
        return False

    def find_frontier_goal(self):
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin
        data = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        # Vectorized frontier detection
        is_free = (data == 0)
        is_unknown = (data == -1)
        is_wall = (data == 100)
        
        # Kernel for 3x3 neighborhood
        kernel = np.ones((3, 3), dtype=int)
        
        # Count neighbors
        # Note: convolve result includes the center cell itself if the kernel has a center
        # We use mode='constant', cval=0 to treat outside as not-unknown and not-wall
        unknown_counts = convolve(is_unknown.astype(int), kernel, mode='constant', cval=0)
        wall_counts = convolve(is_wall.astype(int), kernel, mode='constant', cval=0)
        
        # Apply frontier logic:
        # 1. Cell itself must be free
        # 2. At least 3 unknown neighbors
        # 3. Less than 3 wall neighbors
        frontier_mask = is_free & (unknown_counts >= 3) & (wall_counts < 3)
        
        # Zero out borders to avoid index errors (although vectorization handles it safely, logic usually requires inner area)
        frontier_mask[0, :] = False
        frontier_mask[-1, :] = False
        frontier_mask[:, 0] = False
        frontier_mask[:, -1] = False

        # Label frontiers
        structure = np.ones((3,3), dtype=bool)
        labeled, num_frontiers = label(frontier_mask, structure=structure)

        # Update metrics with frontier count
        self.metrics.count_frontiers(num_frontiers)
        self.get_logger().info(f'Found {num_frontiers} frontiers.')

        # If too few frontiers, consider exploration complete
        if num_frontiers <= self.min_number_frontiers:
            self.get_logger().info(f'Too few valid frontiers: {num_frontiers} total')
            return None

        # Get robot position in map coordinates
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn('Could not get robot position for frontier evaluation')
            return None
        robot_x, robot_y = robot_pos
        rx = int((robot_x - origin.position.x) / resolution)
        ry = int((robot_y - origin.position.y) / resolution)

        # Find the N closest frontiers to the robot
        frontier_infos = []
        for i in range(1, num_frontiers+1):
            ys, xs = np.where(labeled == i)
            if len(xs) == 0:
                continue
            dists = np.hypot(xs - rx, ys - ry)
            min_idx = np.argmin(dists)
            min_dist = dists[min_idx]
            frontier_infos.append((min_dist, i, xs, ys))
        # Sort by distance
        frontier_infos.sort(key=lambda t: t[0])

        # Compute distance from walls for all cells
        wall_mask = (data == 100)
        if np.any(wall_mask):
            dist_from_wall = distance_transform_edt(~wall_mask)
        else:
            dist_from_wall = np.full_like(data, 1000, dtype=float)  # Large value if no walls


        # Try to find a safe goal on a frontier
        goal_x, goal_y, chosen_label = None, None, None
        
        for _, label_id, xs, ys in frontier_infos:
            # Find the point in this frontier farthest from any wall
            frontier_dists = dist_from_wall[ys, xs]
            safe_idxs = np.where(frontier_dists >= self.min_cells_from_wall)[0]
            if len(safe_idxs) == 0:
                continue  # No safe point in this frontier
            
            # Pick the safest (farthest from wall)
            # Iterate through safe points to find one that is not blacklisted
            # Sort safe points by safety (descending)
            sorted_indices = np.argsort(frontier_dists[safe_idxs])[::-1]
            
            candidate_found = False
            for idx in sorted_indices:
                orig_idx = safe_idxs[idx]
                gx, gy = xs[orig_idx], ys[orig_idx]
                
                wx = gx * resolution + origin.position.x
                wy = gy * resolution + origin.position.y
                
                if not self.is_goal_blacklisted(wx, wy):
                    goal_x, goal_y = gx, gy
                    chosen_label = label_id
                    candidate_found = True
                    break
            
            if candidate_found:
                break
        
        # Return if no frontier was selected (because of unsafe distance to walls or blacklist)
        if goal_x is None or goal_y is None:
            self.get_logger().info('No safe and non-blacklisted frontier found.')
            return None

        wx = goal_x * resolution + origin.position.x
        wy = goal_y * resolution + origin.position.y

        # Calculate orientation to face the goal from current position
        target_yaw = 0.0
        if robot_pos is not None:
             target_yaw = math.atan2(wy - robot_y, wx - robot_x)

        # Store the label for later comparison
        self.current_frontier_label = chosen_label

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rclpy.time.Time().to_msg()
        goal.pose.position.x = wx
        goal.pose.position.y = wy
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.pose.orientation.w = math.cos(target_yaw / 2.0)

        return goal

    def send_navigation_goal(self, pose):
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f'Navigating to x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}'
        )

        # Record goal sent
        self.metrics.record_goal_sent()

        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.goal_in_progress = True
        self.goal_start_time = self.get_clock().now()
        self.current_goal_handle = None
        self.current_goal_pose = pose
        self.goal_sent_time = self.get_clock().now()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self.goal_in_progress = False
            self.goal_start_time = None
            return

        #self.get_logger().info('Goal accepted, waiting for result...')
        self.current_goal_handle = goal_handle  # Save handle for cancellation
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        goal_handle = future.result()
        result_status = goal_handle.status

        # Check if the goal was reached (Status 4 is SUCCEEDED)        
        if result_status == GoalStatus.STATUS_SUCCEEDED:
             self.metrics.record_goal_completed()
             # Check if reached too quickly (false frontier)
             if hasattr(self, 'goal_sent_time') and hasattr(self, 'current_goal_pose'):
                elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
                if elapsed < self.min_goal_duration:
                    self.get_logger().info(f'Frontier reached too quickly ({elapsed:.2f}s). Blacklisting.')
                    self.blacklist_goal(
                        self.current_goal_pose.pose.position.x, 
                        self.current_goal_pose.pose.position.y
                    )
        else:
             self.get_logger().info(f'Goal failed with status: {result_status}')
             # Blacklist failed goals (Aborted, etc)
             if hasattr(self, 'current_goal_pose'):
                 self.blacklist_goal(
                     self.current_goal_pose.pose.position.x,
                     self.current_goal_pose.pose.position.y
                 )

        self.goal_in_progress = False
        self.goal_start_time = None
        self.current_goal_handle = None

        self.explore()

    def cancel_current_goal(self):
        if hasattr(self, 'current_goal_handle') and self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Current goal cancelled.'))

def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
