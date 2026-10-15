import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PolygonStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import numpy as np
from scipy.spatial import distance
import time
import math
import os
# Add TF2 imports
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException, TransformException
import rclpy.time
import rclpy.duration
from roboclean.utils import path_planner

class CleanerNode(Node):
    def __init__(self):
        super().__init__('cleaner_node')
        self.global_frame_id = 'map'

        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.cleaning_polygon_sub = self.create_subscription(
            PolygonStamped, '/cleaning_polygon', self.cleaning_polygon_callback, map_qos)
        self.vacuum_map_pub = self.create_publisher(
            OccupancyGrid, '/vacuum_map', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map = None
        self.visited = None
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_handle = None
        self.path_goal_queue = []
        self.path_total_goals = 0
        self.active_clean_mode = None
        self.stamp_timer = self.create_timer(0.5, self.stamp_footprint)  # Stamping every 0.5s
        #self.nav_timer = self.create_timer(2.0, self.clean_step)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.smart_clean_retry_count = 0
        self.smart_clean_timer = None
        self.cleaning_polygon = None
        self.smart_clean_started = False
        self.shutdown_requested = False

        #------parameters
        self.edge_margin_m = 0.40  # keep at least 40cm from map boundaries/obstacles
        self.footprint_size = 0.30  # meters, quadratic footprint (side length)
        self.until_timeout = 10.0
        self.min_dist_to_wall = 0.8  # meters

    def map_callback(self, msg):
        if self.map is None:
            self.map = msg
            width = msg.info.width
            height = msg.info.height
            self.visited = np.zeros((height, width), dtype=np.int8)
            self.get_logger().info(f"try smart clean immediately on first map")
            self.try_smart_clean()  # Start smart cleaning immediately on receiving the first map
        self.publish_vacuum_map()

    def cleaning_polygon_callback(self, msg):
        if not msg.polygon.points:
            self.get_logger().warn('Received empty cleaning polygon, ignoring.')
            return

        self.cleaning_polygon = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        self.get_logger().info(f'Received cleaning polygon with {len(self.cleaning_polygon)} points.')

        if self.map is not None and not self.smart_clean_started:
            self.try_smart_clean()

    def get_robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except (LookupException, TimeoutException, TransformException):
            self.get_logger().warn('TF from map to base_link not available yet.')
            return None

    def meters_to_cells(self, distance_m, *, round_up=False, min_cells=0):
        if self.map is None:
            return min_cells
        res = float(self.map.info.resolution)
        if res <= 0.0:
            return min_cells
        raw = float(distance_m) / res
        cells = int(np.ceil(raw)) if round_up else int(np.floor(raw))
        return max(min_cells, cells)

    def world_to_cell(self, wx, wy):
        map_origin_x = self.map.info.origin.position.x
        map_origin_y = self.map.info.origin.position.y
        res = self.map.info.resolution
        cell_x = int(np.floor((wx - map_origin_x) / res))
        cell_y = int(np.floor((wy - map_origin_y) / res))
        return cell_x, cell_y

    def yaw_to_quaternion(self, yaw):
        half_yaw = 0.5 * float(yaw)
        qx = 0.0
        qy = 0.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return self.normalize_quaternion(qx, qy, qz, qw)

    def normalize_quaternion(self, qx, qy, qz, qw):
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12 or not math.isfinite(norm):
            return 0.0, 0.0, 0.0, 1.0
        return qx / norm, qy / norm, qz / norm, qw / norm

    def resolve_goal_frame_id(self):
        if self.map is not None and self.map.header.frame_id:
            return self.map.header.frame_id
        return self.global_frame_id

    def create_pose_stamped(self, wx, wy, yaw):
        if not (math.isfinite(wx) and math.isfinite(wy) and math.isfinite(yaw)):
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.resolve_goal_frame_id()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(wx)
        pose.pose.position.y = float(wy)
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def compute_segment_yaw(self, start_xy, goal_xy, fallback_yaw=0.0):
        dx = float(goal_xy[0]) - float(start_xy[0])
        dy = float(goal_xy[1]) - float(start_xy[1])
        if math.hypot(dx, dy) < 1e-6:
            return float(fallback_yaw)
        return math.atan2(dy, dx)

    def build_oriented_queue(self, path, start_position=None):
        if not path:
            return []

        points = [(float(wx), float(wy)) for (wx, wy) in path]
        queue = []
        last_yaw = 0.0

        if start_position is not None:
            last_yaw = self.compute_segment_yaw(start_position, points[0], fallback_yaw=0.0)

        for i, (wx, wy) in enumerate(points):
            if i > 0:
                yaw = self.compute_segment_yaw(points[i - 1], points[i], fallback_yaw=last_yaw)
            elif start_position is not None:
                yaw = self.compute_segment_yaw(start_position, points[i], fallback_yaw=last_yaw)
            elif len(points) > 1:
                yaw = self.compute_segment_yaw(points[i], points[i + 1], fallback_yaw=last_yaw)
            else:
                yaw = last_yaw

            pose = self.create_pose_stamped(wx, wy, yaw)
            if pose is None:
                self.get_logger().warn('Skipping non-finite waypoint while building oriented queue.')
                continue
            queue.append(pose)
            last_yaw = yaw

        return queue

    def get_unvisited_cells(self):
        # Returns list of (x, y) cell centers where the footprint can be stamped and is unvisited
        height, width = self.visited.shape
        margin = self.meters_to_cells(self.edge_margin_m, round_up=True, min_cells=0)
        fp_cells = self.meters_to_cells(self.footprint_size * 0.5, round_up=True, min_cells=0)
        data = np.array(self.map.data).reshape((height, width))
        cells = []
        for y in range(margin, height - margin):
            for x in range(margin, width - margin):
                if self.can_stamp(x, y, data, width, height, fp_cells):
                    # Only consider if center cell is not yet visited
                    if self.visited[y, x] == 0:
                        cells.append((x, y))
        return cells

    def can_stamp(self, x, y, data, width, height, fp_cells):
        # Check if the quadratic footprint fits at (x, y) and is at least edge_margin from obstacles
        for dy in range(-fp_cells, fp_cells + 1):
            for dx in range(-fp_cells, fp_cells + 1):
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    return False
                if data[ny, nx] != 0:
                    return False
        return True

    def try_smart_clean(self):
        if self.smart_clean_started:
            return

        if self.cleaning_polygon is None or len(self.cleaning_polygon) < 3:
            self.get_logger().warn('No cleaning polygon set yet. Waiting for /cleaning_polygon.')
            return

        robot_pos = self.get_robot_position()
        if robot_pos is not None:
            started = self.smart_clean()
            if started:
                self.smart_clean_started = True
            if self.smart_clean_timer is not None:
                self.smart_clean_timer.cancel()
                self.smart_clean_timer = None
        else:
            self.smart_clean_retry_count += 1
            if self.smart_clean_retry_count >= 10:
                self.get_logger().error('Robot position still unavailable after 10 attempts, giving up on smart clean.')
                if self.smart_clean_timer is not None:
                    self.smart_clean_timer.cancel()
                    self.smart_clean_timer = None
            else:
                self.get_logger().warn(f'Robot position unavailable, will retry (attempt {self.smart_clean_retry_count}/10)')
                if self.smart_clean_timer is None:
                    self.smart_clean_timer = self.create_timer(0.5, self.try_smart_clean)

    def smart_clean(self):
        self.get_logger().info('Starting smart clean using path planner...')
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn('Robot position unavailable for smart clean.')
            return False
        
        # Load raster-based keepout mask (single source-of-truth for barriers)
        keepout_mask = None
        map_dir = os.getenv("MAP_DIR", "/opt/tmp/map")
        basename = os.getenv("MAP_BASENAME", "roboclean_map")
        keepout_file = os.path.join(map_dir, f"{basename}_keepout.npz")
        if os.path.exists(keepout_file):
            try:
                with np.load(keepout_file) as f:
                    keepout_mask = f["data"].copy()
            except Exception as e:
                self.get_logger().warn(f"Failed to load keepout mask: {e}")

        path_generator = path_planner.CoveragePathGenerator(robot_width=self.footprint_size)
        path, polygons = path_generator.generate_path(
            self.map,
            cleaning_polygon=self.cleaning_polygon,
            keepout_mask=keepout_mask,
        )
        if not path:
            self.get_logger().warn('Path planner returned empty path.')
            return False
        
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return False

        queue = self.build_oriented_queue(path, start_position=robot_pos)
        if not queue:
            self.get_logger().warn('Failed to build oriented goal queue from path.')
            return False

        self.path_goal_queue = queue
        self.path_total_goals = len(queue)
        self.active_clean_mode = 'smart_clean_path'
        self.get_logger().info(f'Prepared {self.path_total_goals} queued NavigateToPose goals from path planner.')
        return self.send_next_queued_goal()

    def send_next_queued_goal(self):
        if self.active_clean_mode != 'smart_clean_path':
            return False

        if not self.path_goal_queue:
            self.get_logger().info('Smart clean path queue finished.')
            self.is_navigating = False
            self.goal_start_time = None
            self.current_goal_handle = None
            self.active_clean_mode = None
            self.request_shutdown_after_clean()
            return True

        pose = self.path_goal_queue.pop(0)
        pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        sent_idx = self.path_total_goals - len(self.path_goal_queue)
        self.get_logger().info(
            f'Sending queued goal {sent_idx}/{self.path_total_goals}: '
            f'x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}'
        )

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        self.goal_start_time = time.time()
        self.current_goal_handle = None
        return True

    def request_shutdown_after_clean(self):
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.get_logger().info('Cleaning complete. Shutting down cleaner node (mode -> idle).')
        if self.smart_clean_timer is not None:
            self.smart_clean_timer.cancel()
            self.smart_clean_timer = None
        rclpy.shutdown()

    def clean_step(self):
        if self.map is None or self.visited is None:
            return

        # Handle goal timeout
        if self.is_navigating and self.goal_start_time is not None:
            elapsed = time.time() - self.goal_start_time
            if elapsed > self.until_timeout:
                self.get_logger().warn(f'Goal timeout: took longer than {self.until_timeout}s, cancelling and choosing new goal.')
                self.cancel_current_goal()
                self.is_navigating = False
                self.goal_start_time = None
                return

        if self.is_navigating:
            return

        unvisited = self.get_unvisited_cells()
        if not unvisited:
            self.get_logger().info('Coverage complete!')
            return

        # Get robot's current position in map cell coordinates
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            self.get_logger().warn('Robot position unavailable, skipping clean_step.')
            return
        rx, ry = robot_pos
        cell_x, cell_y = self.world_to_cell(rx, ry)
        res = self.map.info.resolution
        current_pose = (cell_x, cell_y)

        # Filter unvisited cells to only those further than a minimum distance
        min_goal_distance_cells = self.meters_to_cells(self.min_dist_to_wall, round_up=True, min_cells=1)
        far_unvisited = [
            cell for cell in unvisited
            if distance.euclidean(current_pose, cell) >= min_goal_distance_cells
        ]
        candidate_cells = far_unvisited if far_unvisited else unvisited

        # Find the nearest (far) unvisited cell to current_pose
        dists = [distance.euclidean(current_pose, cell) for cell in candidate_cells]
        next_idx = int(np.argmin(dists))
        x, y = candidate_cells[next_idx]
        goal_x = self.map.info.origin.position.x + x * self.map.info.resolution + self.map.info.resolution / 2.0
        goal_y = self.map.info.origin.position.y + y * self.map.info.resolution + self.map.info.resolution / 2.0
        yaw = self.compute_segment_yaw((rx, ry), (goal_x, goal_y), fallback_yaw=0.0)
        pose = self.create_pose_stamped(goal_x, goal_y, yaw)
        if pose is None:
            self.get_logger().warn('Computed non-finite fallback pose, skipping goal.')
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        self.get_logger().info(f'Navigating to x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}')
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.is_navigating = True
        self.goal_start_time = time.time()
        self.current_goal_handle = None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by server')
            self.is_navigating = False
            self.goal_start_time = None
            self.current_goal_handle = None
            if self.active_clean_mode == 'smart_clean_path':
                self.send_next_queued_goal()
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        # On arrival, update current_goal as visited (robot pose is always from TF)
        self.is_navigating = False
        self.goal_start_time = None
        self.current_goal_handle = None
        if self.active_clean_mode == 'smart_clean_path':
            self.send_next_queued_goal()

    def cancel_current_goal(self):
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info('Current goal cancelled.'))

    def stamp_footprint(self):
        # Stamp the quadratic footprint at the robot's current pose (from TF)
        if self.visited is None or self.map is None:
            return
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return
        rx, ry = robot_pos
        x, y = self.world_to_cell(rx, ry)
        fp_cells = self.meters_to_cells(self.footprint_size * 0.5, round_up=True, min_cells=0)
        height, width = self.visited.shape
        for dy in range(-fp_cells, fp_cells + 1):
            for dx in range(-fp_cells, fp_cells + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    self.visited[ny, nx] = 100
        self.publish_vacuum_map()

    def publish_vacuum_map(self):
        if self.visited is None or self.map is None:
            return
        msg = OccupancyGrid()
        msg.header = self.map.header
        msg.info = self.map.info
        msg.data = list(self.visited.flatten())
        self.vacuum_map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CleanerNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
