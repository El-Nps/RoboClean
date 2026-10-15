import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_period', 0.5)
        self.declare_parameter('publish_count', 20)

        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.publish_count = int(self.get_parameter('publish_count').value)

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.sent_count = 0
        self.timer = self.create_timer(self.publish_period, self._publish_initial_pose)

        self.get_logger().info(
            f'Publishing initial pose to /initialpose: '
            f'x={self.initial_x:.2f}, y={self.initial_y:.2f}, yaw={self.initial_yaw:.2f} rad'
        )

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = Time().to_msg()
        msg.header.frame_id = self.frame_id

        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)

        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.publisher.publish(msg)
        self.sent_count += 1

        if self.sent_count >= self.publish_count:
            self.get_logger().info('Initial pose publishing complete.')
            self.timer.cancel()
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
