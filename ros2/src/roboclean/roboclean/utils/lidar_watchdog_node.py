import rclpy
from rclpy.node import Node
import time
import subprocess
from sensor_msgs.msg import LaserScan

class LidarWatchdog(Node):
    def __init__(self):
        super().__init__('lidar_watchdog')
        self.last_scan_time = time.monotonic()
        self.first_scan_received = False
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )
        self.timer = self.create_timer(1.0, self.check_timeout)
        self.get_logger().info('Lidar watchdog started.')

    def scan_callback(self, msg):
        self.last_scan_time = time.monotonic()
        self.first_scan_received = True

    def check_timeout(self):
        timeout_limit = 5.0 if self.first_scan_received else 20.0
        
        if time.monotonic() - self.last_scan_time > timeout_limit:
            self.get_logger().error('Lidar unresponsive! Restarting...')
            subprocess.run(['pkill', '-15', '-f', 'rplidar_composition'])
            
            self.last_scan_time = time.monotonic() + 15.0
            self.first_scan_received = False


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
