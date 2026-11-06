#!/usr/bin/env python3

"""
A* Global Motion Planner for Retrieval Robot

Subscribes:
    /map       - nav_msgs/msg/OccupancyGrid
    /get_task  - geometry_msgs/msg/PoseStamped

Publishes:
    /planned_path - nav_msgs/msg/Path

Features:
    - A* global path planning
    - Euclidean distance-transform obstacle inflation
    - Configurable robot clearance
    - 8-connected grid search
    - Diagonal corner-cutting prevention
    - Line-of-sight waypoint pruning
    - TF-based robot starting position

Compatible with ROS 2 Jazzy.
"""

import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException,
)


class MotionPlanner(Node):

    def __init__(self):

        super().__init__("motion_planner")

        # ==========================================
        # Parameters
        # ==========================================

        self.declare_parameter(
            "robot_clearance_m",
            1.2
        )

        self.declare_parameter(
            "unknown_is_obstacle",
            True
        )

        self.map_msg = None
        self.blocked = None

        # ==========================================
        # TF Listener
        # ==========================================

        self.tf = Buffer()

        self.listener = TransformListener(
            self.tf,
            self
        )

        # ==========================================
        # QoS
        # ==========================================

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # ==========================================
        # Subscribers
        # ==========================================

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            map_qos
        )

        self.task_sub = self.create_subscription(
            PoseStamped,
            "/get_task",
            self.on_task,
            10
        )

        # ==========================================
        # Publisher
        # ==========================================

        self.path_pub = self.create_publisher(
            Path,
            "/planned_path",
            map_qos
        )

        self.get_logger().info(
            "Motion planner started. Waiting for /map and /get_task."
        )

    # ==============================================
    # MAP CALLBACK
    # ==============================================

    def on_map(self, msg):

        # Ignore repeated static maps with identical dimensions.
        if (
            self.map_msg is not None
            and self.blocked is not None
            and self.map_msg.info.width == msg.info.width
            and self.map_msg.info.height == msg.info.height
            and self.map_msg.info.resolution == msg.info.resolution
        ):
            return

        self.get_logger().info(
            "Received occupancy map."
        )

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        self.get_logger().info(
            f"Map dimensions: {width} x {height}"
        )

        self.get_logger().info(
            f"Map resolution: {resolution:.4f} m/cell"
        )

        # ==========================================
        # Convert occupancy grid to NumPy array
        # ==========================================

        grid = np.asarray(
            msg.data,
            dtype=np.int16
        ).reshape((height, width))

        unknown_is_obstacle = bool(
            self.get_parameter(
                "unknown_is_obstacle"
            ).value
        )

        blocked = grid >= 50

        if unknown_is_obstacle:
            blocked |= grid < 0

        # ==========================================
        # Get obstacle inflation radius
        # ==========================================

        clearance = float(
            self.get_parameter(
                "robot_clearance_m"
            ).value
        )

        if clearance < 0:
            self.get_logger().error(
                "robot_clearance_m must be nonnegative."
            )
            return

        self.get_logger().info(
            f"Inflating obstacles by {clearance:.2f} meters."
        )

        # ==========================================
        # MEMORY-EFFICIENT OBSTACLE INFLATION
        # ==========================================

        try:

            if clearance > 0:

                # For every free cell, calculate the
                # Euclidean distance to the nearest
                # occupied cell.
                #
                # sampling=resolution converts
                # the distances to meters.

                distance_map = distance_transform_edt(
                    ~blocked,
                    sampling=resolution
                )

                # Any cell at or within the
                # clearance radius becomes blocked.

                inflated = distance_map <= clearance

                del distance_map

            else:

                inflated = blocked.copy()

            # Commit only after successful processing.

            self.blocked = inflated
            self.map_msg = msg

            occupied_cells = int(
                np.count_nonzero(self.blocked)
            )

            total_cells = int(
                self.blocked.size
            )

            self.get_logger().info(
                f"Obstacle inflation complete. "
                f"Blocked cells: {occupied_cells}/{total_cells}"
            )

            self.get_logger().info(
                "Motion planner ready to receive tasks."
            )

        except MemoryError:

            self.get_logger().error(
                "Insufficient memory to inflate map."
            )

        except Exception as exc:

            self.get_logger().error(
                f"Map processing failed: {exc}"
            )

    # ==============================================
    # WORLD <-> GRID TRANSFORMS
    # ==============================================

    def world_to_cell(self, x, y):

        m = self.map_msg.info

        # Map origin orientation is assumed to be zero.

        cell_x = int(
            math.floor(
                (x - m.origin.position.x) / m.resolution
            )
        )

        cell_y = int(
            math.floor(
                (y - m.origin.position.y) / m.resolution
            )
        )

        return (cell_x, cell_y)

    def cell_to_world(self, cell):

        m = self.map_msg.info

        x = (
            m.origin.position.x
            + (cell[0] + 0.5) * m.resolution
        )

        y = (
            m.origin.position.y
            + (cell[1] + 0.5) * m.resolution
        )

        return (x, y)

    # ==============================================
    # CHECK WHETHER CELL IS FREE
    # ==============================================

    def free(self, cell):

        if self.blocked is None:
            return False

        x, y = cell

        height, width = self.blocked.shape

        if not (0 <= x < width and 0 <= y < height):
            return False

        return not bool(self.blocked[y, x])

    # ==============================================
    # A* PATH PLANNING
    # ==============================================

    def find_path(self, start, goal):

        neighbors = [
            (dx, dy, math.hypot(dx, dy))
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx != 0 or dy != 0
        ]

        queue = [
            (0.0, start)
        ]

        g_cost = {
            start: 0.0
        }

        parent = {}

        closed = set()

        while queue:

            _, current = heapq.heappop(queue)

            if current in closed:
                continue

            if current == goal:

                path = [current]

                while current in parent:

                    current = parent[current]

                    path.append(current)

                path.reverse()

                return path

            closed.add(current)

            for dx, dy, movement_cost in neighbors:

                next_cell = (
                    current[0] + dx,
                    current[1] + dy
                )

                if not self.free(next_cell):
                    continue

                # Prevent diagonal corner cutting.

                if dx != 0 and dy != 0:

                    side_1 = (
                        current[0] + dx,
                        current[1]
                    )

                    side_2 = (
                        current[0],
                        current[1] + dy
                    )

                    if (
                        not self.free(side_1)
                        or not self.free(side_2)
                    ):
                        continue

                new_cost = (
                    g_cost[current]
                    + movement_cost
                )

                if new_cost < g_cost.get(
                    next_cell,
                    float("inf")
                ):

                    g_cost[next_cell] = new_cost

                    parent[next_cell] = current

                    heuristic = math.hypot(
                        goal[0] - next_cell[0],
                        goal[1] - next_cell[1]
                    )

                    priority = (
                        new_cost + heuristic
                    )

                    heapq.heappush(
                        queue,
                        (priority, next_cell)
                    )

        return []

    # ==============================================
    # LINE-OF-SIGHT CHECK
    # ==============================================

    def visible(self, start, end):

        steps = (
            max(
                abs(start[0] - end[0]),
                abs(start[1] - end[1])
            ) * 3 + 1
        )

        for i in range(steps + 1):

            t = i / max(steps, 1)

            x = round(
                start[0] * (1 - t)
                + end[0] * t
            )

            y = round(
                start[1] * (1 - t)
                + end[1] * t
            )

            if not self.free((x, y)):
                return False

        return True

    # ==============================================
    # WAYPOINT PRUNING
    # ==============================================

    def shorten(self, path):

        if not path:
            return []

        short_path = [
            path[0]
        ]

        i = 0

        while i < len(path) - 1:

            j = len(path) - 1

            while (
                j > i + 1
                and not self.visible(
                    path[i],
                    path[j]
                )
            ):
                j -= 1

            short_path.append(
                path[j]
            )

            i = j

        return short_path

    # ==============================================
    # PUBLISH PLANNED PATH
    # ==============================================

    def publish_path(self, cells, task):

        path_msg = Path()

        path_msg.header.frame_id = "map"

        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for cell in cells:

            x, y = self.cell_to_world(cell)

            pose = PoseStamped()

            pose.header = path_msg.header

            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        # Preserve the requested orientation
        # at the final destination.

        if path_msg.poses:

            path_msg.poses[-1].pose.orientation = (
                task.pose.orientation
            )

        self.path_pub.publish(
            path_msg
        )

    # ==============================================
    # TASK CALLBACK
    # ==============================================

    def on_task(self, task):

        self.get_logger().info(
            "New navigation task received."
        )

        # Clear the previous path immediately.
        self.publish_path([], task)

        if task.header.frame_id != "map":

            self.get_logger().error(
                "Goal must use the map frame."
            )

            return

        if self.blocked is None:

            self.get_logger().error(
                "Map not ready. Resend task."
            )

            return

        # ==========================================
        # Get current robot position from TF
        # ==========================================

        try:

            transform = self.tf.lookup_transform(
                "map",
                "base_link",
                Time()
            )

        except TransformException as exc:

            self.get_logger().error(
                f"Localization TF unavailable: {exc}"
            )

            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        goal_x = task.pose.position.x
        goal_y = task.pose.position.y

        self.get_logger().info(
            f"Robot: ({robot_x:.2f}, {robot_y:.2f}) | "
            f"Goal: ({goal_x:.2f}, {goal_y:.2f})"
        )

        # ==========================================
        # Convert positions to grid cells
        # ==========================================

        start = self.world_to_cell(
            robot_x,
            robot_y
        )

        goal = self.world_to_cell(
            goal_x,
            goal_y
        )

        if not self.free(start):

            self.get_logger().error(
                f"Robot start {start} is inside "
                "an inflated obstacle."
            )

            return

        if not self.free(goal):

            self.get_logger().error(
                f"Goal {goal} is inside "
                "an inflated obstacle or outside map."
            )

            return

        # ==========================================
        # A* planning
        # ==========================================

        self.get_logger().info(
            "Running A* path planning..."
        )

        route = self.find_path(
            start,
            goal
        )

        if not route:

            self.get_logger().error(
                "A* failed: no traversable path."
            )

            return

        self.get_logger().info(
            f"A* generated {len(route)} grid cells."
        )

        # ==========================================
        # Reduce waypoints
        # ==========================================

        route = self.shorten(
            route
        )

        # ==========================================
        # Publish trajectory
        # ==========================================

        self.publish_path(
            route,
            task
        )

        self.get_logger().info(
            f"Planned path published: "
            f"{len(route)} waypoints."
        )


# ==============================================
# MAIN
# ==============================================

def main(args=None):

    rclpy.init(args=args)

    node = MotionPlanner()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
