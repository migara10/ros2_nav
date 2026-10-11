import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.03  # must match the bridge wheel_radius


class TurnCalib(Node):
    def __init__(self):
        super().__init__('turn_calib')
        self.create_subscription(JointState, '/joint_states', self.cb, 10)
        self.base = None
        self.count = 0

    def cb(self, msg):
        pos = dict(zip(msg.name, msg.position))
        left = pos.get('left_wheel_joint')
        right = pos.get('right_wheel_joint')
        if left is None or right is None:
            return
        if self.base is None:
            self.base = (left, right)
            return
        diff = WHEEL_RADIUS * ((right - self.base[1]) - (left - self.base[0]))
        d = abs(diff)
        self.count += 1
        if self.count % 5 == 0:
            text = (
                f'wheel diff={diff:+.3f} m | wheel_sep if physical turn was: '
                f'90={d / math.radians(90):.3f}  '
                f'180={d / math.radians(180):.3f}  '
                f'360={d / math.radians(360):.3f}  '
                f'720={d / math.radians(720):.3f}  '
                f'1080={d / math.radians(1080):.3f}   '
            )
            print(text, end='\r', flush=True)


def main():
    rclpy.init()
    node = TurnCalib()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()