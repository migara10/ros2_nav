import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

# these three must match the values currently used by the bridge
WHEEL_RADIUS = 0.0335
CURRENT_TPR = 1473.0
CURRENT_GAIN = 1.25


class CalibCheck(Node):
    def __init__(self):
        super().__init__('calib_check')
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.x0 = None
        self.y0 = None
        self.x = 0.0
        self.y = 0.0
        self.prev_yaw = None
        self.yaw_total = 0.0
        self.base = None
        self.left = 0.0
        self.right = 0.0
        self.create_timer(0.5, self.show)

    def odom_cb(self, msg):
        p = msg.pose.pose
        if self.x0 is None:
            self.x0 = p.position.x
            self.y0 = p.position.y
        self.x = p.position.x - self.x0
        self.y = p.position.y - self.y0
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        if self.prev_yaw is not None:
            d = yaw - self.prev_yaw
            while d > math.pi:
                d -= 2.0 * math.pi
            while d < -math.pi:
                d += 2.0 * math.pi
            self.yaw_total += d
        self.prev_yaw = yaw

    def joint_cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        left = pos.get('left_wheel_joint')
        right = pos.get('right_wheel_joint')
        if left is None or right is None:
            return
        if self.base is None:
            self.base = (left, right)
        self.left = left - self.base[0]
        self.right = right - self.base[1]

    def show(self):
        if self.base is None or self.x0 is None:
            print('waiting for /odom and /joint_states ...', flush=True)
            return

        left_m = WHEEL_RADIUS * self.left
        right_m = WHEEL_RADIUS * self.right
        travel = (left_m + right_m) / 2.0
        diff = right_m - left_m
        dist = math.hypot(self.x, self.y)

        lines = []
        lines.append(
            f'odom : x={self.x:+.3f} m  y={self.y:+.3f} m  '
            f'distance={dist:.3f} m  yaw={math.degrees(self.yaw_total):+.1f} deg'
        )
        lines.append(
            f'wheel: left={left_m:+.3f} m  right={right_m:+.3f} m  '
            f'mean travel={travel:+.3f} m  (right-left)={diff:+.3f} m'
        )
        lines.append('')

        lines.append('STRAIGHT test  (if the real distance was ...)')
        if abs(travel) > 0.05:
            for real in (0.5, 1.0, 2.0):
                lines.append(
                    f'   {real:.1f} m  ->  ticks_per_rev = {CURRENT_TPR * abs(travel) / real:.0f}'
                )
        else:
            lines.append('   (drive forward first)')

        if left_m * right_m > 0 and min(abs(left_m), abs(right_m)) > 0.1:
            ratio = right_m / left_m
            lines.append(
                f'   right/left travel = {ratio:.3f}  ->  left_gain = {CURRENT_GAIN * ratio:.3f}'
            )
        lines.append('')

        lines.append('TURN test  (if the real turn was ...)')
        if abs(diff) > 0.05:
            for deg in (90, 180, 360, 720, 1080):
                lines.append(
                    f'  {deg:>4} deg  ->  wheel_sep = {abs(diff) / math.radians(deg):.3f} m'
                )
        else:
            lines.append('   (turn the robot first)')

        print('\033[H\033[J' + '\n'.join(lines), flush=True)


def main():
    rclpy.init()
    node = CalibCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()