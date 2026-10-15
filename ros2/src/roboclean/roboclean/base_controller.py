import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray, String
import serial
import math
import tf2_ros

class BaseController(Node):

    def __init__(self):
        super().__init__('base_controller')

        # Configuration
        self.declare_parameter('serial_port', '/dev/sensors/arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_separation', 0.25)
        self.declare_parameter('wheel_radius', 0.035)
        self.declare_parameter('pulses_per_rotation', 519)
        self.declare_parameter('max_rpm', 60.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Arduino Serial Connection
        self.serial = None
        self.connect_serial()

        # ROS Interfaces
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32MultiArray, '/cleaning_commands', self.cleaning_callback, 10)
        self.create_subscription(String, '/sensor_req', self.sensor_req_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.sensor_pub = self.create_publisher(Int32MultiArray, '/proximity_sensors', 10)
        self.sensor_data_pub = self.create_publisher(String, '/sensor_data', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Internal State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        # Accumulators for the current time step
        self.tick_d_center = 0.0
        self.tick_d_theta = 0.0

        self.last_time = self.get_clock().now()

        # Timer: 20 Hz (0.05s)
        self.create_timer(0.05, self.update_loop)


    def connect_serial(self):
        """Attempt to connect to the serial port."""

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value

        try:
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().debug(f"Could not connect to Arduino: {e}")
            self.serial = None


    def cmd_vel_callback(self, msg):
        """Convert ROS Twist (v, w) -> Target Ticks per 50ms for Left/Right Wheels."""

        if not self.serial or not self.serial.is_open:
            return

        sep = self.get_parameter('wheel_separation').value
        radius = self.get_parameter('wheel_radius').value
        ppr = self.get_parameter('pulses_per_rotation').value

        # v = (vl + vr)/2, w = (vr - vl)/sep
        vl_m_s = msg.linear.x - (msg.angular.z * sep / 2.0)
        vr_m_s = msg.linear.x + (msg.angular.z * sep / 2.0)
        
        # Convert meters per second to Ticks per Second
        ticks_per_m = ppr / (2.0 * math.pi * radius)
        vl_ticks_sec = vl_m_s * ticks_per_m
        vr_ticks_sec = vr_m_s * ticks_per_m

        # Convert Ticks per Second to Ticks per 50ms interval (0.05 seconds)
        # This matches the SEND_INTERVAL loop running on the Arduino
        target_ticks_L = vl_ticks_sec * 0.05
        target_ticks_R = vr_ticks_sec * 0.05
    
        self.send_serial_command(target_ticks_L, target_ticks_R)


    def send_serial_command(self, targetL, targetR):
        """Send target ticks to Arduino."""

        try:
            # Send as floats with 2 decimal precision
            cmd = f"M,{targetL:.2f},{targetR:.2f}\n"
            self.serial.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")
            self.serial.close()
            self.serial = None


    def cleaning_callback(self, msg):
        """Handle Vacuum/Brush commands: [vacuum_state, brush_state]"""
        if not self.serial or not self.serial.is_open:
            return

        if len(msg.data) >= 2:
            vac = msg.data[0]
            brush = msg.data[1]
            try:
                self.serial.write(f"V,{vac}\n".encode())
                self.serial.write(f"B,{brush}\n".encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")
                self.serial.close()
                self.serial = None


    def sensor_req_callback(self, msg):
        """Handle single sensor read request: 'type,pin'"""
        if not self.serial or not self.serial.is_open:
            return

        # msg.data ex: "a,0" or "d,12"
        parts = msg.data.split(',')
        if len(parts) == 2:
            cmd = f"R,{parts[0]},{parts[1]}\n"
            try:
                self.serial.write(cmd.encode())
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")
                self.serial.close()
                self.serial = None


    def update_loop(self):
        """
        Main Control Loop:
        1. Auto-reconnect if needed.
        2. Read ALL available serial packets (accumulate distance).
        3. Calculate velocity over the specific timer interval (dt).
        4. Publish Odometry (always, even if 0).
        """

        if self.serial is None or not self.serial.is_open:
            self.connect_serial()
            # If still invalid, publish 0 velocity to keep Nav2 alive
            if self.serial is None:
                self.publish_odometry(0.0, 0.0)
                self.last_time = self.get_clock().now()
                return

        # Read and Accumulate Data
        self.tick_d_center = 0.0
        self.tick_d_theta = 0.0
        data_received = False

        try:
            # Process all waiting packets to catch up
            while self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()

                if line.startswith('E'):
                    parts = line.split(',')
                    if len(parts) >= 3:
                        tick_delta_left = int(parts[1])
                        tick_delta_right = int(parts[2])
                        self.integrate_position(tick_delta_left, tick_delta_right)
                        data_received = True

                elif line.startswith('SA') or line.startswith('SD'):
                    # Single sensor response: "SA,pin,val" or "SD,pin,val"
                    parts = line.split(',')
                    if len(parts) == 3:
                        key = f"{parts[0]}-{parts[1]}"
                        val = parts[2]
                        self.sensor_data_pub.publish(String(data=f"{key}: {val}"))

                elif line.startswith('S,'):
                    # Proximity data: "S,left,mid,right"
                    parts = line[2:].split(',')
                    if len(parts) == 3:
                        msg = Int32MultiArray()
                        msg.data = [int(parts[0]), int(parts[1]), int(parts[2])]
                        self.sensor_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.serial.close()
            self.serial = None

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0:
            if data_received:
                # Average velocity over this time step
                vx = self.tick_d_center / dt
                vth = self.tick_d_theta / dt
                self.publish_odometry(vx, vth)
            else:
                # No data received (connection idle or silent), assume 0 velocity
                self.publish_odometry(0.0, 0.0)


    def integrate_position(self, delta_ticks_l, delta_ticks_r):
        """Calculate new position based on wheel tick deltas."""

        sep = self.get_parameter('wheel_separation').value
        radius = self.get_parameter('wheel_radius').value
        ppr = self.get_parameter('pulses_per_rotation').value

        d_left = (delta_ticks_l / ppr) * 2.0 * math.pi * radius
        d_right = (delta_ticks_r / ppr) * 2.0 * math.pi * radius

        d_center = (d_left + d_right) / 2.0    # distance traveled (center)                  
        d_theta = (d_right - d_left) / sep     # change in angle

        self.tick_d_center += d_center
        self.tick_d_theta += d_theta

        # Update pose (Global Frame)
        dx = d_center * math.cos(self.th + d_theta / 2.0)
        dy = d_center * math.sin(self.th + d_theta / 2.0)

        self.x += dx
        self.y += dy
        self.th += d_theta

        # Normalize
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))


    def publish_odometry(self, vx, vth):
        current_time = self.get_clock().now()
        odom_frame = self.get_parameter('odom_frame').value
        base_frame = self.get_parameter('base_frame').value

        q = self.euler_to_quaternion(0, 0, self.th)

        # Publish Transform (TF)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = odom_frame
        t.child_frame_id = base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)


    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
