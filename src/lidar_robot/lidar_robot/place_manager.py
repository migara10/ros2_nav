import argparse
import math
import os
import sys
import time

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

DEFAULT_FILE = os.path.expanduser('~/ros2_nav/places.yaml')


def load_places(path):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    return data.get('places', {}) or {}


def save_places(path, places):
    with open(path, 'w') as f:
        yaml.safe_dump({'places': places}, f, default_flow_style=False)


class PlaceNode(Node):
    def __init__(self):
        super().__init__('place_manager')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.last_print = 0.0

    def get_pose(self, timeout=5.0):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time()
                )
            except Exception:
                continue
            q = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return t.transform.translation.x, t.transform.translation.y, yaw
        return None

    def feedback_cb(self, feedback_msg):
        now = time.time()
        if now - self.last_print > 2.0:
            self.last_print = now
            remaining = feedback_msg.feedback.distance_remaining
            print(f'   distance remaining: {remaining:.2f} m')

    def go_to(self, name, x, y, yaw):
        if not self.client.wait_for_server(timeout_sec=5.0):
            print('Nav2 navigate_to_pose server not available. Is Nav2 running?')
            return False

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        print(f'Going to "{name}" (x={x:.2f}, y={y:.2f})')
        send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            print(f'Goal "{name}" was rejected by Nav2')
            return False

        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            print('Cancelling the goal')
            handle.cancel_goal_async()
            raise

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print(f'Arrived at "{name}"')
            return True
        print(f'Could not reach "{name}" (status {status})')
        return False


def main(args=None):
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)[1:]

    parser = argparse.ArgumentParser(prog='places', description='Named places for Nav2')
    parser.add_argument('--file', default=DEFAULT_FILE, help='places yaml file')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('list', help='show saved places')

    p_save = sub.add_parser('save', help='save the current robot pose as a place')
    p_save.add_argument('name')

    p_delete = sub.add_parser('delete', help='delete a saved place')
    p_delete.add_argument('name')

    p_go = sub.add_parser('go', help='go to one or more places in order')
    p_go.add_argument('names', nargs='+')
    p_go.add_argument('--repeat', type=int, default=1)

    opts = parser.parse_args(argv)
    path = os.path.expanduser(opts.file)
    places = load_places(path)

    if opts.command == 'list':
        if not places:
            print(f'No places saved yet ({path})')
        for name, p in places.items():
            print(f'{name}: x={p["x"]:.2f} y={p["y"]:.2f} yaw={math.degrees(p["yaw"]):.0f} deg')
        rclpy.shutdown()
        return

    if opts.command == 'delete':
        if opts.name in places:
            del places[opts.name]
            save_places(path, places)
            print(f'Deleted "{opts.name}"')
        else:
            print(f'No place named "{opts.name}"')
        rclpy.shutdown()
        return

    node = PlaceNode()
    try:
        if opts.command == 'save':
            pose = node.get_pose()
            if pose is None:
                print('Could not read the robot pose (map -> base_footprint). Is Nav2 running?')
            else:
                x, y, yaw = pose
                places[opts.name] = {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
                save_places(path, places)
                print(f'Saved "{opts.name}": x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')
        elif opts.command == 'go':
            missing = [n for n in opts.names if n not in places]
            if missing:
                print(f'Unknown place(s): {", ".join(missing)}')
                print(f'Known places: {", ".join(places) or "none"}')
            else:
                for round_no in range(opts.repeat):
                    if opts.repeat > 1:
                        print(f'--- round {round_no + 1} of {opts.repeat} ---')
                    ok = True
                    for name in opts.names:
                        p = places[name]
                        ok = node.go_to(name, p['x'], p['y'], p['yaw'])
                        if not ok:
                            break
                    if not ok:
                        print('Stopping because a goal failed')
                        break
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()