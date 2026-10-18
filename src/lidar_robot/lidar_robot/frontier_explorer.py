import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('min_frontier_cells', 6)
        self.declare_parameter('clearance', 0.25)
        self.declare_parameter('min_goal_distance', 0.4)
        self.declare_parameter('blacklist_radius', 0.4)
        self.declare_parameter('goal_timeout', 90.0)
        self.declare_parameter('size_weight', 0.5)
        self.declare_parameter('max_runtime_min', 30.0)
        self.declare_parameter('empty_checks_to_finish', 5)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_cells = self.get_parameter('min_frontier_cells').value
        self.clearance = self.get_parameter('clearance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.size_weight = self.get_parameter('size_weight').value
        self.max_runtime = self.get_parameter('max_runtime_min').value * 60.0
        self.empty_limit = self.get_parameter('empty_checks_to_finish').value

        self.map_msg = None
        self.blacklist = []
        self.navigating = False
        self.goal_handle = None
        self.goal_xy = None
        self.goal_start = None
        self.empty_count = 0
        self.finished = False
        self.start_time = self.get_clock().now()
        self.waited_for_map = False

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self.map_cb, qos
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(1.0, self.tick)

        self.get_logger().info('Frontier explorer started, waiting for the map and Nav2')

    def map_cb(self, msg):
        self.map_msg = msg

    def get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            if math.hypot(x - bx, y - by) < self.blacklist_radius:
                return True
        return False

    def find_frontiers(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int16).reshape((h, w))

        unknown = data == -1
        free = (data >= 0) & (data < 50)
        occupied = data >= 50

        # a frontier cell is a free cell that touches an unknown cell
        pad = np.pad(unknown, 1, constant_values=False)
        near_unknown = (
            pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
        )
        frontier = free & near_unknown

        visited = np.zeros_like(frontier, dtype=bool)
        clusters = []
        ys, xs = np.nonzero(frontier)
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if visited[y0, x0]:
                continue
            queue = deque([(y0, x0)])
            visited[y0, x0] = True
            cells = []
            while queue:
                y, x = queue.popleft()
                cells.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny = y + dy
                        nx = x + dx
                        if (
                            0 <= ny < h
                            and 0 <= nx < w
                            and frontier[ny, nx]
                            and not visited[ny, nx]
                        ):
                            visited[ny, nx] = True
                            queue.append((ny, nx))
            if len(cells) >= self.min_cells:
                clusters.append(cells)
        return clusters, occupied

    def has_clearance(self, occupied, y, x, radius):
        h, w = occupied.shape
        y0 = max(0, y - radius)
        y1 = min(h, y + radius + 1)
        x0 = max(0, x - radius)
        x1 = min(w, x + radius + 1)
        return not occupied[y0:y1, x0:x1].any()

    def choose_goal(self, msg, rx, ry):
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        clusters, occupied = self.find_frontiers(msg)
        radius = int(math.ceil(self.clearance / res))

        best = None
        for cells in clusters:
            cy = sum(c[0] for c in cells) / len(cells)
            cx = sum(c[1] for c in cells) / len(cells)
            ordered = sorted(cells, key=lambda c: (c[0] - cy) ** 2 + (c[1] - cx) ** 2)

            goal_cell = None
            for y, x in ordered[:40]:
                if self.has_clearance(occupied, y, x, radius):
                    goal_cell = (y, x)
                    break
            if goal_cell is None:
                continue

            gx = ox + (goal_cell[1] + 0.5) * res
            gy = oy + (goal_cell[0] + 0.5) * res
            dist = math.hypot(gx - rx, gy - ry)
            if dist < self.min_goal_distance:
                continue
            if self.is_blacklisted(gx, gy):
                continue

            size_m = len(cells) * res
            score = dist - self.size_weight * size_m
            if best is None or score < best[0]:
                best = (score, gx, gy)
        return best, len(clusters)

    def send_goal(self, gx, gy, rx, ry):
        yaw = math.atan2(gy - ry, gx - rx)
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_xy = (gx, gy)
        self.goal_start = self.get_clock().now()
        self.navigating = True
        self.get_logger().info(f'Going to frontier at x={gx:.2f} y={gy:.2f}')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected, blacklisting this frontier')
            self.blacklist.append(self.goal_xy)
            self.navigating = False
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached frontier')
        else:
            self.get_logger().warn(f'Goal ended with status {status}, blacklisting it')
            self.blacklist.append(self.goal_xy)
        self.goal_handle = None
        self.navigating = False

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def finish(self, text):
        self.finished = True
        self.cancel_goal()
        self.get_logger().info(text)

    def tick(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.max_runtime:
            self.finish('Maximum run time reached, stopping exploration')
            return

        if self.navigating:
            goal_age = (self.get_clock().now() - self.goal_start).nanoseconds / 1e9
            if goal_age > self.goal_timeout:
                self.get_logger().warn('Goal timed out, cancelling and blacklisting it')
                self.blacklist.append(self.goal_xy)
                self.cancel_goal()
                self.navigating = False
            return

        if self.map_msg is None:
            if not self.waited_for_map:
                self.get_logger().info('Waiting for /map')
                self.waited_for_map = True
            return

        if not self.client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('Waiting for the Nav2 navigate_to_pose server')
            return

        pose = self.get_robot_xy()
        if pose is None:
            self.get_logger().info('Waiting for the map to base_footprint transform')
            return
        rx, ry = pose

        best, count = self.choose_goal(self.map_msg, rx, ry)
        if best is None:
            self.empty_count += 1
            self.get_logger().info(
                f'No reachable frontier ({count} frontier groups seen), '
                f'check {self.empty_count}/{self.empty_limit}'
            )
            if self.empty_count >= self.empty_limit:
                self.finish('Exploration complete. Save the map now.')
            return

        self.empty_count = 0
        _, gx, gy = best
        self.send_goal(gx, gy, rx, ry)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cancel_goal()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()