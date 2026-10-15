import rclpy
from sensor_msgs.msg import LaserScan

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('wait_for_scan')
    node.get_logger().info('Waiting for first laser scan data before starting Nav2...')
    future = rclpy.Future()
    
    def cb(msg):
        node.get_logger().info('First scan received. Exiting block to trigger Nav2 startup.')
        future.set_result(True)
        
    sub = node.create_subscription(LaserScan, '/scan', cb, 1)
    
    # Block until the first scan is received and the future is resolved
    rclpy.spin_until_future_complete(node, future)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()