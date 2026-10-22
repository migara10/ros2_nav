import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanGate(Node):
    """Republish /scan to /scan_gated only while the robot is not rotating fast."""

    def __init__(self):
        super().__init__('scan_gate')

        self.declare_parameter('input_topic', 'scan')
        self.declare_parameter('output_topic', 'scan_gated')
        self.declare_parameter('angular_threshold', 0.2)  # rad/s
        self.declare_parameter('hold_off', 0.35)  # seconds

        self.angular_threshold = self.get_parameter('angular_threshold').value
        self.hold_off = self.get_parameter('hold_off').value

        self.last_fast_time = None
        self.passed = 0
        self.dropped = 0
        self.last_report = self.get_clock().now()

        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value, 10
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter('input_topic').value,
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)

        self.get_logger().info(
            f'Scan gate ready: dropping scans while |angular velocity| > '
            f'{self.angular_threshold} rad/s (+{self.hold_off} s)'
        )

    def odom_cb(self, msg):
        if abs(msg.twist.twist.angular.z) > self.angular_threshold:
            self.last_fast_time = self.get_clock().now()

    def scan_cb(self, msg):
        now = self.get_clock().now()
        blocked = False
        if self.last_fast_time is not None:
            age = (now - self.last_fast_time).nanoseconds / 1e9
            blocked = age < self.hold_off

        if blocked:
            self.dropped += 1
        else:
            self.passed += 1
            self.pub.publish(msg)

        if (now - self.last_report).nanoseconds / 1e9 > 10.0:
            self.get_logger().info(
                f'scans passed={self.passed} dropped={self.dropped}'
            )
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()