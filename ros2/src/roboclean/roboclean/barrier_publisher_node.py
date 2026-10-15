import os
import json
import time
import math

import numpy as np
from skimage.draw import polygon as sk_polygon

import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from tf2_ros import Buffer, TransformListener, LookupException, TimeoutException, TransformException


class BarrierPublisherNode(Node):
    def __init__(self):
        super().__init__('barrier_publisher_node')
        
        self.declare_parameter('map_dir', '/opt/tmp/map')
        self.declare_parameter('map_basename', 'roboclean_map')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        self.filter_info_pub = self.create_publisher(CostmapFilterInfo, '/costmap_filter_info', qos_profile)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/keepout_filter_mask', qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.create_timer(2.0, self.timer_callback)
        self.create_subscription(Int32MultiArray, '/proximity_sensors', self.proximity_callback, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_ledge_time = 0.0
        
        self.last_mtime_npz = -1.0
        self.last_mtime_json = -1.0
        self.published_empty = False

        self.map_info = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)

    def map_callback(self, msg):
        self.map_info = msg.info

    def proximity_callback(self, msg):
        threshold = int(os.getenv("LEDGE_THRESHOLD", "100"))
        if len(msg.data) >= 3:
            left, mid, right = msg.data[0], msg.data[1], msg.data[2]
            if left < threshold or mid < threshold or right < threshold:
                # Instantly halt the robot
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.cmd_pub.publish(stop_msg)

                current_time = time.time()
                # Debounce automatic ledge keepout generation
                if current_time - self.last_ledge_time > float(os.getenv("LEDGE_DEBOUNCE_S", "2.0")):
                    self.last_ledge_time = current_time
                    try:
                        self.mark_ledge_barrier()
                    except Exception as e:
                        self.get_logger().warn(f"Failed to mark ledge barrier: {e}")

    def _world_to_grid(self, wx, wy):
        res = self.map_info.resolution
        gx = (float(wx) - float(self.map_info.origin.position.x)) / res
        gy = (float(wy) - float(self.map_info.origin.position.y)) / res
        return int(gx), int(gy)

    def timer_callback(self):
        if self.map_info is None:
            return

        map_dir = self.get_parameter('map_dir').value
        basename = self.get_parameter('map_basename').value
        
        npz_file = os.path.join(map_dir, f"{basename}_keepout.npz")
        
        if not os.path.exists(npz_file):
            if not self.published_empty:
                self.publish_empty_mask()
                self.published_empty = True
                self.last_mtime_npz = -1.0
            return
            
        mtime_npz = os.path.getmtime(npz_file) if os.path.exists(npz_file) else -1.0
        
        if mtime_npz != self.last_mtime_npz:
            self.last_mtime_npz = mtime_npz
            self.published_empty = False
            self.build_and_publish_mask(npz_file)

    def build_and_publish_mask(self, npz_file):
        w, h = self.map_info.width, self.map_info.height
        
        # 1. Load raster data from NPZ
        try:
            with np.load(npz_file) as npz:
                data = npz['data'].copy()
            if data.shape != (h, w):
                self.get_logger().warn("NPZ mask dimensions mismatch, regenerating...")
                data = np.zeros((h, w), dtype=np.int8)
        except Exception as e:
            self.get_logger().warn(f"Failed to read barriers npz: {e}")
            data = np.zeros((h, w), dtype=np.int8)

        # 2. Publish the grid
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = self.map_info
        grid.data = data.flatten().tolist()
        
        self.mask_pub.publish(grid)
        self.publish_info()

    def mark_ledge_barrier(self):
        # Create a small rectangular keepout in front of the robot and save to npz
        if self.map_info is None:
            self.get_logger().warn('Map info not available for ledge barrier creation')
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, TimeoutException, TransformException):
            self.get_logger().warn('TF from map to base_link not available for ledge barrier')
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        theta = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        cx = x + float(os.getenv("LEDGE_OFFSET_M", "0.15")) * math.cos(theta)
        cy = y + float(os.getenv("LEDGE_OFFSET_M", "0.15")) * math.sin(theta)

        hw = float(os.getenv("LEDGE_HW_M", "0.1"))
        hd = float(os.getenv("LEDGE_HD_M", "0.05"))
        corners = [
            (hd, hw),
            (hd, -hw),
            (-hd, -hw),
            (-hd, hw),
        ]

        map_dir = self.get_parameter('map_dir').value
        basename = self.get_parameter('map_basename').value
        npz_file = os.path.join(map_dir, f"{basename}_keepout.npz")

        w, h = int(self.map_info.width), int(self.map_info.height)
        res = float(self.map_info.resolution)
        ox = float(self.map_info.origin.position.x)
        oy = float(self.map_info.origin.position.y)

        try:
            with np.load(npz_file) as f:
                grid = f['data'].copy()
            if grid.shape != (h, w):
                grid = np.zeros((h, w), dtype=np.int8)
        except Exception:
            grid = np.zeros((h, w), dtype=np.int8)

        px = []
        py = []
        for l in corners:
            wx = cx + l[0] * math.cos(theta) - l[1] * math.sin(theta)
            wy = cy + l[0] * math.sin(theta) + l[1] * math.cos(theta)
            gx = (wx - ox) / res
            gy = (wy - oy) / res
            px.append(gx)
            py.append(gy)

        rr, cc = sk_polygon(py, px, shape=grid.shape)
        grid[rr, cc] = int(os.getenv("KEEPOUT_FILL", "100"))

        try:
            np.savez_compressed(npz_file, data=grid)
        except Exception as e:
            self.get_logger().warn(f"Failed to write keepout npz: {e}")
            return

        # Immediately publish updated mask
        try:
            self.build_and_publish_mask(npz_file)
        except Exception as e:
            self.get_logger().warn(f"Failed to publish keepout mask after ledge: {e}")

    def publish_empty_mask(self):
        if not self.map_info:
            return
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info = self.map_info
        grid.data = [0] * (self.map_info.width * self.map_info.height)
        self.mask_pub.publish(grid)
        self.publish_info()

    def publish_info(self):
        info = CostmapFilterInfo()
        info.header.frame_id = 'map'
        info.header.stamp = self.get_clock().now().to_msg()
        info.type = 0 # Keepout filter
        info.filter_mask_topic = "/keepout_filter_mask"
        info.base = 0.0
        info.multiplier = 1.0
        self.filter_info_pub.publish(info)

def main(args=None):
    rclpy.init(args=args)
    node = BarrierPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
