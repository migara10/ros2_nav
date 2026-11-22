import math
import os
import subprocess
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker


def find_frontier_clusters(data, width, height, clearance_cells, min_size):
    """Return a list of clusters, each a list of (row, col) frontier cells.

    A frontier cell is a free cell that touches an unknown cell. Cells closer
    than clearance_cells to an occupied cell are dropped, because the robot
    cannot stand there anyway.
    """
    grid = np.asarray(data, dtype=np.int16).reshape(height, width)
    unknown = grid < 0
    free = (grid >= 0) & (grid < 50)
    occupied = grid >= 50

    unknown_nb = np.zeros_like(unknown)
    unknown_nb[1:, :] |= unknown[:-1, :]
    unknown_nb[:-1, :] |= unknown[1:, :]
    unknown_nb[:, 1:] |= unknown[:, :-1]
    unknown_nb[:, :-1] |= unknown[:, 1:]
    frontier = free & unknown_nb

    near = occupied.copy()
    for _ in range(clearance_cells):
        grown = near.copy()
        grown[1:, :] |= near[:-1, :]
        grown[:-1, :] |= near[1:, :]
        grown[:, 1:] |= near[:, :-1]
        grown[:, :-1] |= near[:, 1:]
        grown[1:, 1:] |= near[:-1, :-1]
        grown[1:, :-1] |= near[:-1, 1:]
        grown[:-1, 1:] |= near[1:, :-1]
        grown[:-1, :-1] |= near[1:, 1:]
        near = grown
    frontier &= ~near

    rows, cols = np.nonzero(frontier)
    cells = set(zip(rows.tolist(), cols.tolist()))
    clusters = []
    while cells:
        start = cells.pop()
        members = [start]
        queue = deque([start])
        while queue:
            cr, cc = queue.popleft()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nb = (cr + dr, cc + dc)
                    if nb in cells:
                        cells.remove(nb)
                        queue.append(nb)
                        members.append(nb)
        if len(members) >= min_size:
            clusters.append(members)
    return clusters


def cell_to_world(row, col, res, ox, oy):
    return ox + (col + 0.5) * res, oy + (row + 0.5) * res


def choose_frontier(clusters, robot_xy, res, ox, oy, min_goal_dist,
                    blacklist, blacklist_radius, gain_scale):
    """Pick the best frontier goal. Returns (score, x, y, size) or None."""
    best = None
    for members in clusters:
        mean_r = sum(m[0] for m in members) / len(members)
        mean_c = sum(m[1] for m in members) / len(members)
        # use a real frontier cell (the one nearest to the centroid) as the goal
        mr, mc = min(members, key=lambda m: (m[0] - mean_r) ** 2 + (m[1] - mean_c) ** 2)
        gx, gy = cell_to_world(mr, mc, res, ox, oy)
        dist = math.hypot(gx - robot_xy[0], gy - robot_xy[1])
        if dist < min_goal_dist:
            continue
        if any(math.hypot(gx - bx, gy - by) < blacklist_radius for bx, by in blacklist):
            continue
        score = dist - gain_scale * len(members) * res
        if best is None or score < best[0]:
            best = (score, gx, gy, len(members))
    return best


def goal_still_frontier(clusters, goal_xy, res, ox, oy, radius):
    for members in clusters:
        for r, c in members:
            x, y = cell_to_world(r, c, res, ox, oy)
            if math.hypot(x - goal_xy[0], y - goal_xy[1]) < radius:
                return True
    return False


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('clearance', 0.25)
        self.declare_parameter('min_goal_distance', 0.35)
        self.declare_parameter('goal_timeout', 120.0)
        self.declare_parameter('blacklist_radius', 0.4)
        self.declare_parameter('gain_scale', 1.0)
        self.declare_parameter('idle_ticks_to_finish', 6)
        self.declare_parameter('return_to_start', True)
        self.declare_parameter('map_dir', '~/ros2_nav/src/lidar_robot/maps')

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.min_size = self.get_parameter('min_frontier_size').value
        self.clearance = self.get_parameter('clearance').value
        self.min_goal_dist = self.get_parameter('min_goal_distance').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.gain_scale = self.get_parameter('gain_scale').value
        self.idle_limit = self.get_parameter('idle_ticks_to_finish').value
        self.return_to_start = self.get_parameter('return_to_start').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self.map_cb, map_qos)
        self.marker_pub = self.create_publisher(Marker, 'explore/frontiers', 1)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.map_msg = None
        self.start_pose = None
        self.phase = 'explore'  # explore -> returning -> done
        self.goal_active = False
        self.goal_handle = None
        self.current_goal = None
        self.goal_start = 0.0
        self.last_check = 0.0
        self.blacklist = []
        self.blacklist_after_cancel = False
        self.cancel_sent = False
        self.idle_ticks = 0

        self.create_timer(1.0, self.tick)
        self.get_logger().info('Frontier explorer started, waiting for /map and Nav2')

    # ------------------------------------------------------------ helpers

    def map_cb(self, msg):
        self.map_msg = msg

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def current_clusters(self):
        m = self.map_msg
        res = m.info.resolution
        clearance_cells = max(1, int(round(self.clearance / res)))
        return find_frontier_clusters(
            m.data, m.info.width, m.info.height, clearance_cells, self.min_size)

    def publish_markers(self, clusters):
        m = self.map_msg
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontiers'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.g = 1.0
        marker.color.a = 1.0
        for members in clusters:
            for r, c in members:
                x, y = cell_to_world(r, c, res, ox, oy)
                marker.points.append(Point(x=x, y=y, z=0.05))
        self.marker_pub.publish(marker)

    # --------------------------------------------------------- navigation

    def send_goal(self, gx, gy, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_active = True
        self.cancel_sent = False
        self.current_goal = (gx, gy)
        self.goal_start = time.monotonic()
        self.last_check = time.monotonic()
        self.nav_client.send_goal_async(goal).add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected, blacklisting it')
            if self.current_goal is not None and self.phase == 'explore':
                self.blacklist.append(self.current_goal)
            self.goal_active = False
            self.current_goal = None
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        goal = self.current_goal
        self.goal_active = False
        self.goal_handle = None
        self.current_goal = None
        self.cancel_sent = False

        if self.phase == 'returning':
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Back at the start pose. Exploration complete.')
            else:
                self.get_logger().warn('Could not return to the start pose.')
            self.phase = 'done'
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Reached the frontier goal')
        elif status == GoalStatus.STATUS_CANCELED:
            if self.blacklist_after_cancel and goal is not None:
                self.blacklist.append(goal)
        elif goal is not None:
            self.get_logger().warn('Goal failed, blacklisting it')
            self.blacklist.append(goal)
        self.blacklist_after_cancel = False

    def cancel_goal(self, blacklist):
        if self.goal_handle is None or self.cancel_sent:
            return
        self.cancel_sent = True
        self.blacklist_after_cancel = blacklist
        self.goal_handle.cancel_goal_async()

    # ------------------------------------------------------------ main loop

    def tick(self):
        if self.phase == 'done':
            return
        if self.map_msg is None:
            return
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.0)
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info(
                'Start pose (%.2f, %.2f), exploring' % (pose[0], pose[1]))

        if self.phase == 'returning':
            return

        clusters = self.current_clusters()
        self.publish_markers(clusters)

        res = self.map_msg.info.resolution
        ox = self.map_msg.info.origin.position.x
        oy = self.map_msg.info.origin.position.y

        if self.goal_active:
            now = time.monotonic()
            if self.goal_handle is not None and now - self.goal_start > self.goal_timeout:
                self.get_logger().warn('Goal timeout, giving up on it')
                self.cancel_goal(True)
            elif (self.goal_handle is not None and self.current_goal is not None
                  and now - self.last_check > 3.0):
                self.last_check = now
                if not goal_still_frontier(
                        clusters, self.current_goal, res, ox, oy, 0.5):
                    self.get_logger().info('Frontier already explored, picking a new one')
                    self.cancel_goal(False)
            return

        target = choose_frontier(
            clusters, pose[:2], res, ox, oy, self.min_goal_dist,
            self.blacklist, self.blacklist_radius, self.gain_scale)

        if target is None:
            self.idle_ticks += 1
            if self.idle_ticks >= self.idle_limit:
                self.finish(pose)
            return

        self.idle_ticks = 0
        _, gx, gy, size = target
        yaw = math.atan2(gy - pose[1], gx - pose[0])
        self.get_logger().info(
            'Going to frontier (%.2f, %.2f), %d cells' % (gx, gy, size))
        self.send_goal(gx, gy, yaw)

    def finish(self, pose):
        self.get_logger().info('No reachable frontiers left, saving the map')
        self.save_map()
        if self.return_to_start and self.start_pose is not None:
            self.phase = 'returning'
            self.get_logger().info('Returning to the start pose')
            self.send_goal(self.start_pose[0], self.start_pose[1], self.start_pose[2])
        else:
            self.phase = 'done'
            self.get_logger().info('Exploration complete.')

    def save_map(self):
        directory = os.path.expanduser(self.get_parameter('map_dir').value)
        os.makedirs(directory, exist_ok=True)
        name = datetime.now().strftime('auto_map_%Y%m%d_%H%M%S')
        path = os.path.join(directory, name)
        try:
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path],
                timeout=60, check=False)
        except Exception as exc:
            self.get_logger().error('Map saver failed: %s' % exc)
            return
        if os.path.exists(path + '.yaml'):
            self.get_logger().info('Map saved as %s.yaml' % path)
        else:
            self.get_logger().error('Map file was not created, save it manually')


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()