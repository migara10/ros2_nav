import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomCheck(Node):
    def __init__(self):
        super().__init__('odom_check')
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.prev_yaw = None
        self.total = 0.0
        self.count = 0

    def cb(self, msg):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        if self.prev_yaw is not None:
            d = yaw - self.prev_yaw
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            self.total += d
        self.prev_yaw = yaw
        self.count += 1
        if self.count % 5 == 0:
            print(
                f'x={p.position.x:.3f} m   y={p.position.y:.3f} m   '
                f'turned={math.degrees(self.total):.1f} deg      ',
                end='\r', flush=True,
            )


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()