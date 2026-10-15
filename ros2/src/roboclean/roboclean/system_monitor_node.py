import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import json

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(String, '/system_metrics', 10)
        self.timer = self.create_timer(1.0, self.publish_metrics) # 1 Hz
        self.get_logger().info('System Monitor Node started')

    def publish_metrics(self):
        try:
            metrics = {
                'cpu_percent': psutil.cpu_percent(interval=None),
                'ram_percent': psutil.virtual_memory().percent,
                'disk_percent': psutil.disk_usage('/').percent
            }
            msg = String()
            msg.data = json.dumps(metrics)
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read system metrics: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
