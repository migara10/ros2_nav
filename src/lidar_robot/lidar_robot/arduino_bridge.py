import math
import time

import rclpy
import serial
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('ticks_per_rev', 1403.0)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_sep', 0.175)
        self.declare_parameter('max_wheel_speed', 0.25)
        self.declare_parameter('min_pwm', 130)
        self.declare_parameter('left_gain', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)

        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_sep').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.min_pwm = self.get_parameter('min_pwm').value
        self.left_gain = self.get_parameter('left_gain').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(2.0)  # Uno resets when the port opens
        self.ser.reset_input_buffer()

        self.rx_buffer = ''
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.prev_left = None
        self.prev_right = None
        self.prev_time = None
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)

        self.create_timer(0.01, self.read_serial)
        self.create_timer(0.1, self.send_command)

        self.get_logger().info(f'Connected to Arduino on {port}')

    def cmd_vel_cb(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def to_pwm(self, v):
        # map |v| in (0, max_wheel_speed] to PWM in [min_pwm, 255]
        if abs(v) < 1e-3:
            return 0
        ratio = min(abs(v) / self.max_wheel_speed, 1.0)
        pwm = int(self.min_pwm + (255 - self.min_pwm) * ratio)
        return pwm if v > 0 else -pwm

    def send_command(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if age > self.cmd_timeout:
            v, w = 0.0, 0.0
        else:
            v, w = self.cmd_v, self.cmd_w

        v_left = v - w * self.wheel_sep / 2.0
        v_right = v + w * self.wheel_sep / 2.0

        pwm_l = self.to_pwm(v_left)
        pwm_r = self.to_pwm(v_right)
        pwm_l = int(max(-255, min(255, pwm_l * self.left_gain)))

        line = f'L{pwm_l} R{pwm_r}\n'
        self.ser.write(line.encode())

    def read_serial(self):
        data = self.ser.read(self.ser.in_waiting or 1)
        if not data:
            return
        self.rx_buffer += data.decode(errors='ignore')
        while '\n' in self.rx_buffer:
            line, self.rx_buffer = self.rx_buffer.split('\n', 1)
            line = line.strip()
            if not line.startswith('E'):
                continue
            parts = line[1:].split()
            if len(parts) != 2:
                continue
            try:
                left = int(parts[0])
                right = int(parts[1])
            except ValueError:
                continue
            self.update_odometry(left, right)

    def update_odometry(self, left, right):
        now = self.get_clock().now()
        if self.prev_left is None:
            self.prev_left, self.prev_right, self.prev_time = left, right, now
            return

        dt = (now - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        meters_per_tick = 2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
        d_left = (left - self.prev_left) * meters_per_tick
        d_right = (right - self.prev_right) * meters_per_tick
        self.prev_left, self.prev_right, self.prev_time = left, right, now

        ds = (d_left + d_right) / 2.0
        dth = (d_right - d_left) / self.wheel_sep

        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th += dth

        self.left_pos += d_left / self.wheel_radius
        self.right_pos += d_right / self.wheel_radius

        stamp = now.to_msg()
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dth / dt
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos, self.right_pos]
        self.joint_pub.publish(js)

    def destroy_node(self):
        try:
            self.ser.write(b'L0 R0\n')
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()