"""
frontier_explorer_node.py
  - Frontier cell: FREE (0) with at least one 4-connected UNKNOWN (-1) neighbour.
  - 8-connected BFS clustering.
  - Goal = nearest valid cluster centroid to robot pose.
"""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import tf2_ros


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
UNKNOWN = -1
# 4-connected offsets for frontier detection
FOUR_CONN = ((1, 0), (-1, 0), (0, 1), (0, -1))
# 8-connected offsets for clustering
EIGHT_CONN = ((1, 0), (-1, 0), (0, 1), (0, -1),
              (1, 1), (1, -1), (-1, 1), (-1, -1))


class FrontierExplorerNode(Node):
    """Autonomous frontier exploration node."""

    def __init__(self):
        super().__init__('frontier_explorer_node')

        # ---- Declare parameters ----
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('refresh_period_sec', 1.0)
        self.declare_parameter('min_cluster_cells', 25)
        self.declare_parameter('frontier_size_weight', 0.0)
        self.declare_parameter('free_thresh', 50)
        self.declare_parameter('max_goal_dist_m', 8.0)
        self.declare_parameter('min_goal_dist_m', 0.3)
        self.declare_parameter('goal_blacklist_radius_m', 0.5)
        self.declare_parameter('goal_blacklist_sec', 60.0)
        self.declare_parameter('goal_reached_blacklist_sec', 120.0)
        self.declare_parameter('preempt_score_ratio', 2.0)
        self.declare_parameter('preempt_cooldown_sec', 60.0)
        self.declare_parameter('heading_bonus', 1.5)
        self.declare_parameter('manual_blacklist_radius_m', 1.0)
        self.declare_parameter('debug_log', False)

        # ---- Read parameters ----
        self._map_topic = self.get_parameter('map_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._robot_frame = self.get_parameter('robot_base_frame').value
        self._refresh_sec = self.get_parameter('refresh_period_sec').value
        self._min_cluster = self.get_parameter('min_cluster_cells').value
        self._size_weight = self.get_parameter('frontier_size_weight').value
        self._free_thresh = self.get_parameter('free_thresh').value
        self._max_goal_dist = self.get_parameter('max_goal_dist_m').value
        self._min_goal_dist = self.get_parameter('min_goal_dist_m').value
        self._bl_radius = self.get_parameter('goal_blacklist_radius_m').value
        self._bl_sec = self.get_parameter('goal_blacklist_sec').value
        self._bl_reached_sec = self.get_parameter('goal_reached_blacklist_sec').value
        self._preempt_ratio = self.get_parameter('preempt_score_ratio').value
        self._preempt_cooldown = self.get_parameter('preempt_cooldown_sec').value
        self._heading_bonus = self.get_parameter('heading_bonus').value
        self._manual_bl_radius = self.get_parameter('manual_blacklist_radius_m').value
        self._debug = self.get_parameter('debug_log').value

        # ---- State ----
        self._map_msg = None          # Latest OccupancyGrid
        self._blacklist = []          # List of (x, y, expiry_time)
        self._manual_blacklist = []   # List of (x, y) — permanent, from RViz
        self._navigating = False      # True while a Nav2 goal is active
        self._current_goal = None     # (x, y) of active goal
        self._current_score = 0.0     # Score of the active goal
        self._goal_sent_time = 0.0    # monotonic time when goal was sent
        self._goal_handle = None
        self._cancelling = False      # True while cancelling a goal
        self._map_diag_done = False   # One-shot map diagnostics flag

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- Map subscription (TRANSIENT_LOCAL for Cartographer latched map) ----
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, self._map_topic, self._map_cb, map_qos
        )

        # ---- Nav2 action client ----
        self._nav_cb_group = MutuallyExclusiveCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._nav_cb_group,
        )

        # ---- Visualization publisher ----
        self._marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)

        # ---- Manual blacklist via RViz "Publish Point" tool ----
        self.create_subscription(
            PointStamped, '/clicked_point', self._clicked_point_cb, 10
        )

        # ---- Main loop timer ----
        self.create_timer(self._refresh_sec, self._explore_tick)

        self.get_logger().info(
            f'Frontier explorer started  |  map={self._map_topic}  '
            f'frame={self._global_frame}->{self._robot_frame}  '
            f'period={self._refresh_sec}s'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _clicked_point_cb(self, msg: PointStamped):
        """Add a permanent no-go zone from RViz Publish Point tool."""
        x = msg.point.x
        y = msg.point.y
        self._manual_blacklist.append((x, y))
        self.get_logger().info(
            f'MANUAL BLACKLIST added at ({x:.2f}, {y:.2f}), '
            f'radius={self._manual_bl_radius:.1f}m  '
            f'(total {len(self._manual_blacklist)} zones)'
        )

    def _map_cb(self, msg: OccupancyGrid):
        """Store the latest map."""
        self._map_msg = msg

        # One-shot diagnostics: log cell value distribution on first map
        if not self._map_diag_done:
            self._map_diag_done = True
            data = msg.data
            n = len(data)
            n_unk = sum(1 for v in data if v == UNKNOWN)
            n_free = sum(1 for v in data if 0 <= v < self._free_thresh)
            n_occ = sum(1 for v in data if v >= self._free_thresh)
            # Sample some unique values for debug
            sample = set()
            for v in data:
                sample.add(int(v))
                if len(sample) > 20:
                    break
            self.get_logger().info(
                f'MAP DIAG: {msg.info.width}x{msg.info.height}, '
                f'total={n}, unknown({UNKNOWN})={n_unk}, '
                f'free(0..{self._free_thresh - 1})={n_free}, '
                f'occupied({self._free_thresh}..100)={n_occ}, '
                f'sample_values={sorted(sample)}'
            )

        if self._debug:
            self.get_logger().debug(
                f'Map updated: {msg.info.width}x{msg.info.height}, '
                f'res={msg.info.resolution:.3f}'
            )

    # ------------------------------------------------------------------
    # Main exploration tick
    # ------------------------------------------------------------------
    def _explore_tick(self):
        """Called periodically to drive exploration."""

        # 1. If a cancel is in-flight, wait for it to finish
        if self._cancelling:
            if self._debug:
                self.get_logger().debug('Cancel in progress, skipping tick.')
            return

        # 2. Need a map
        if self._map_msg is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return

        # 3. Get robot pose (x, y, yaw)
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            return  # logged inside helper
        rx, ry, robot_yaw = robot_pose

        # 4. Prune expired blacklist entries
        now = time.monotonic()
        self._blacklist = [
            (bx, by, exp) for bx, by, exp in self._blacklist if exp > now
        ]

        # 5. Detect frontiers
        map_msg = self._map_msg
        info = map_msg.info
        w, h = info.width, info.height
        data = map_msg.data  # row-major list

        if w == 0 or h == 0:
            self.get_logger().warn('Empty map, skipping.', throttle_duration_sec=5.0)
            return

        frontier_mask = self._build_frontier_mask(data, w, h, self._free_thresh)

        # 6. Cluster and filter
        clusters = self._cluster_frontiers(frontier_mask, w, h)
        clusters = self._filter_clusters(clusters)
        if self._debug:
            self.get_logger().debug(f'Found {len(clusters)} frontier clusters (after filtering).')

        if not clusters:
            self.get_logger().info(
                'No frontiers found — exploration may be complete.',
                throttle_duration_sec=10.0,
            )
            return

        # 7. Convert cluster centroids to world coordinates & score
        ox = info.origin.position.x
        oy = info.origin.position.y
        res = info.resolution

        all_centroids = []   # For visualization: (wx, wy, n_cells)
        candidates = []
        for cells in clusters:
            cx = sum(c % w for c in cells) / len(cells)
            cy = sum(c // w for c in cells) / len(cells)
            wx = ox + (cx + 0.5) * res
            wy = oy + (cy + 0.5) * res
            n_cells = len(cells)
            all_centroids.append((wx, wy, n_cells))
            dist = math.hypot(wx - rx, wy - ry)

            # Filter: distance limits
            if dist > self._max_goal_dist or dist < self._min_goal_dist:
                continue
            # Filter: blacklist
            if self._is_blacklisted(wx, wy):
                continue

            # Score: bigger clusters beat closer small ones
            score = (n_cells ** self._size_weight) / (dist + 0.1)

            # Heading bonus: prefer frontiers in front hemisphere
            angle_to_goal = math.atan2(wy - ry, wx - rx)
            angle_diff = abs(math.atan2(
                math.sin(angle_to_goal - robot_yaw),
                math.cos(angle_to_goal - robot_yaw),
            ))
            if angle_diff < math.pi / 2.0:  # Within ±90°
                score *= self._heading_bonus

            candidates.append((score, dist, wx, wy, n_cells))

        if not candidates:
            self.get_logger().info(
                'No valid frontier candidates after filtering.',
                throttle_duration_sec=10.0,
            )
            return

        # 8. Pick highest-scoring frontier
        candidates.sort(key=lambda t: t[0], reverse=True)
        best_score, best_dist, gx, gy, n_cells = candidates[0]

        # 9. If already navigating, check if current goal is still valid
        if self._navigating:
            cg = self._current_goal
            # Check if current goal's frontier still exists
            if cg is not None:
                goal_still_valid = False
                for cwx, cwy, _ in all_centroids:
                    if (cwx - cg[0]) ** 2 + (cwy - cg[1]) ** 2 < self._bl_radius ** 2:
                        goal_still_valid = True
                        break
                if not goal_still_valid:
                    self.get_logger().info(
                        f'Frontier at ({cg[0]:.2f},{cg[1]:.2f}) closed up — switching goal.'
                    )
                    self._cancel_and_replan(gx, gy, best_score)
                    return

            # Cooldown: don't preempt for a better goal too quickly
            elapsed = time.monotonic() - self._goal_sent_time
            if elapsed < self._preempt_cooldown:
                if self._debug:
                    self.get_logger().debug(
                        f'Cooldown: {elapsed:.0f}/{self._preempt_cooldown:.0f}s — keeping current goal.'
                    )
                return
            if best_score > self._current_score * self._preempt_ratio:
                self.get_logger().info(
                    f'Preempting: better frontier ({gx:.2f},{gy:.2f}) '
                    f'score={best_score:.1f} vs current={self._current_score:.1f}'
                )
                self._cancel_and_replan(gx, gy, best_score)
            else:
                if self._debug:
                    self.get_logger().debug(
                        f'Navigating (score={self._current_score:.1f}), '
                        f'best alternative={best_score:.1f} — keeping current goal.'
                    )
            return

        if self._debug:
            self.get_logger().info(
                f'Selected goal ({gx:.2f}, {gy:.2f}), '
                f'score={best_score:.1f}, dist={best_dist:.2f}m, cells={n_cells}'
            )

        # 10. Publish visualization & send goal
        self._publish_markers(all_centroids, gx, gy)
        self._send_nav_goal(gx, gy, best_score)

    # ------------------------------------------------------------------
    # Frontier detection
    # ------------------------------------------------------------------
    @staticmethod
    def _build_frontier_mask(data, w, h, free_thresh):
        """Return a set of linear indices that are frontier cells.

        Frontier cell: 0 <= data[idx] < free_thresh (i.e. "free") and
        at least one cell within 2 cells is UNKNOWN (-1).

        Cartographer places an occupied border between free and unknown,
        so we must look past that 1-2 cell thick wall.
        """
        # Precompute offsets for a 5x5 search window (radius 2)
        search_offsets = []
        for sy in range(-2, 3):
            for sx in range(-2, 3):
                if sx == 0 and sy == 0:
                    continue
                search_offsets.append((sx, sy))

        frontiers = set()
        for y in range(h):
            row_base = y * w
            for x in range(w):
                idx = row_base + x
                val = data[idx]
                if val < 0 or val >= free_thresh:
                    continue
                # Check within 2-cell radius for unknown
                for dx, dy in search_offsets:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if data[ny * w + nx] == UNKNOWN:
                            frontiers.add(idx)
                            break
        return frontiers

    @staticmethod
    def _cluster_frontiers(frontier_set, w, h):
        """BFS 8-connected clustering.  Returns list of clusters (each a list
        of linear indices) that meet the minimum size threshold.

        NOTE: min_cluster filtering is done by caller for flexibility, but
        we do return all clusters here.
        """
        visited = set()
        clusters = []
        for seed in frontier_set:
            if seed in visited:
                continue
            cluster = []
            queue = deque()
            queue.append(seed)
            visited.add(seed)
            while queue:
                idx = queue.popleft()
                cluster.append(idx)
                cx, cy = idx % w, idx // w
                for dx, dy in EIGHT_CONN:
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < w and 0 <= ny < h:
                        nidx = ny * w + nx
                        if nidx in frontier_set and nidx not in visited:
                            visited.add(nidx)
                            queue.append(nidx)
            clusters.append(cluster)
        return clusters

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _get_robot_pose(self):
        """Return (x, y, yaw) of the robot in the global frame, or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5),
            )
            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            q = tf.transform.rotation
            # Extract yaw from quaternion (only need z-axis rotation)
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return (tx, ty, yaw)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed ({self._global_frame}->{self._robot_frame}): {e}',
                throttle_duration_sec=5.0,
            )
            return None

    def _is_blacklisted(self, x, y):
        """Check if (x,y) is within blacklist radius of any active entry."""
        r2 = self._bl_radius * self._bl_radius
        for bx, by, _ in self._blacklist:
            if (x - bx) ** 2 + (y - by) ** 2 < r2:
                return True
        # Also check permanent manual blacklist (from RViz)
        mr2 = self._manual_bl_radius * self._manual_bl_radius
        for bx, by in self._manual_blacklist:
            if (x - bx) ** 2 + (y - by) ** 2 < mr2:
                return True
        return False

    def _blacklist_point(self, x, y, duration_sec):
        """Add a point to the blacklist."""
        self._blacklist.append((x, y, time.monotonic() + duration_sec))

    # ------------------------------------------------------------------
    # Nav2 interaction
    # ------------------------------------------------------------------
    def _cancel_and_replan(self, new_x, new_y, new_score):
        """Cancel current goal and send a new one once cancellation completes."""
        if self._goal_handle is None:
            self._send_nav_goal(new_x, new_y, new_score)
            return

        self._cancelling = True
        self._navigating = False

        cancel_future = self._goal_handle.cancel_goal_async()

        def _on_cancel_done(future):
            self._cancelling = False
            self._goal_handle = None
            self._current_goal = None
            self._current_score = 0.0
            self.get_logger().info('Previous goal cancelled, sending new goal.')
            self._send_nav_goal(new_x, new_y, new_score)

        cancel_future.add_done_callback(_on_cancel_done)

    def _send_nav_goal(self, x, y, score=0.0):
        """Send a NavigateToPose goal to Nav2."""
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 action server not available.',
                                   throttle_duration_sec=10.0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = self._global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # face forward

        self.get_logger().info(f'Sending goal -> ({x:.2f}, {y:.2f})  score={score:.1f}')
        self._navigating = True
        self._current_goal = (x, y)
        self._current_score = score
        self._goal_sent_time = time.monotonic()

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_cb,
        )
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """Called when the goal is accepted or rejected."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self._navigating = False
            # Blacklist so we don't immediately retry the same spot
            if self._current_goal:
                self._blacklist_point(*self._current_goal, self._bl_sec)
            return

        if self._debug:
            self.get_logger().debug('Goal accepted by Nav2.')
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        """Called when navigation finishes (success, failure, cancel)."""
        result = future.result()
        status = result.status
        gx, gy = self._current_goal or (0.0, 0.0)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Goal reached ({gx:.2f}, {gy:.2f}). '
                f'Blacklisting for {self._bl_reached_sec}s.'
            )
            self._blacklist_point(gx, gy, self._bl_reached_sec)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Goal canceled ({gx:.2f}, {gy:.2f}).')
            self._blacklist_point(gx, gy, self._bl_sec)
        else:
            self.get_logger().warn(
                f'Goal failed ({gx:.2f}, {gy:.2f}), status={status}. '
                f'Blacklisting for {self._bl_sec}s.'
            )
            self._blacklist_point(gx, gy, self._bl_sec)

        self._navigating = False
        self._goal_handle = None
        self._current_goal = None
        self._current_score = 0.0

    def _nav_feedback_cb(self, feedback_msg):
        """Optional: log navigation progress."""
        if self._debug:
            fb = feedback_msg.feedback
            self.get_logger().debug(
                f'Nav feedback — ETA: {fb.estimated_time_remaining.sec}s, '
                f'dist: {fb.distance_remaining:.2f}m'
            )

    # ------------------------------------------------------------------
    # Cluster filtering (called in explore_tick via list comprehension)
    # ------------------------------------------------------------------
    def _filter_clusters(self, clusters):
        """Remove clusters smaller than min_cluster_cells."""
        return [c for c in clusters if len(c) >= self._min_cluster]

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------
    def _publish_markers(self, centroids, goal_x, goal_y):
        """Publish MarkerArray: blue spheres for centroids, green for goal."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # First: delete-all marker to clear previous frame
        del_marker = Marker()
        del_marker.header.frame_id = self._global_frame
        del_marker.header.stamp = stamp
        del_marker.ns = 'frontiers'
        del_marker.action = Marker.DELETEALL
        ma.markers.append(del_marker)

        for i, (wx, wy, n_cells) in enumerate(centroids):
            m = Marker()
            m.header.frame_id = self._global_frame
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.15
            m.pose.orientation.w = 1.0
            # Scale by cluster size (clamped)
            s = min(0.15 + n_cells * 0.005, 0.8)
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s
            m.color = ColorRGBA(r=0.2, g=0.5, b=1.0, a=0.8)
            m.lifetime.sec = 3  # auto-expire
            ma.markers.append(m)

        # Highlight selected goal as green cube
        gm = Marker()
        gm.header.frame_id = self._global_frame
        gm.header.stamp = stamp
        gm.ns = 'frontiers'
        gm.id = len(centroids) + 1
        gm.type = Marker.CUBE
        gm.action = Marker.ADD
        gm.pose.position.x = goal_x
        gm.pose.position.y = goal_y
        gm.pose.position.z = 0.3
        gm.pose.orientation.w = 1.0
        gm.scale.x = 0.35
        gm.scale.y = 0.35
        gm.scale.z = 0.35
        gm.color = ColorRGBA(r=0.1, g=1.0, b=0.2, a=0.9)
        gm.lifetime.sec = 3
        ma.markers.append(gm)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down frontier explorer.')
    finally:
        # Cancel active goal on shutdown
        if node._goal_handle is not None:
            node.get_logger().info('Cancelling active navigation goal...')
            node._goal_handle.cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
