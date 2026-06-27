#!/usr/bin/env python3
"""RViz visualization helper for Nav2 MPPI navigation.

Publishes a compact set of RViz markers for robot pose, heading, velocity,
status text, and the executed trail. Nav2 and MPPI still publish their own
map, costmap, global path, and sampled trajectory topics.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class MppiRvizVisualizer(Node):
    def __init__(self):
        super().__init__("mppi_rviz_visualizer")

        self.declare_parameter("frame_id", "nav_map")
        self.declare_parameter("pose_topic", "/odin1/relocation")
        self.declare_parameter("raw_cmd_topic", "/cmd_vel")
        self.declare_parameter("adjusted_cmd_topic", "/cmd_vel_adjusted")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("trail_max_points", 1000)
        self.declare_parameter("trail_min_distance", 0.02)
        self.declare_parameter("publish_frequency_hz", 10.0)
        self.declare_parameter("robot_radius", 0.30)
        self.declare_parameter("heading_arrow_length", 0.70)
        self.declare_parameter("velocity_arrow_scale", 0.60)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.raw_cmd_topic = str(self.get_parameter("raw_cmd_topic").value)
        self.adjusted_cmd_topic = str(self.get_parameter("adjusted_cmd_topic").value)
        self.plan_topic = str(self.get_parameter("plan_topic").value)
        self.trail_max_points = int(self.get_parameter("trail_max_points").value)
        self.trail_min_distance = float(self.get_parameter("trail_min_distance").value)
        self.robot_radius = float(self.get_parameter("robot_radius").value)
        self.heading_arrow_length = float(self.get_parameter("heading_arrow_length").value)
        self.velocity_arrow_scale = float(self.get_parameter("velocity_arrow_scale").value)

        self.current_pose = None
        self.current_yaw = 0.0
        self.raw_cmd = Twist()
        self.adjusted_cmd = Twist()
        self.global_plan = Path()
        self.global_costmap_info = "global costmap: waiting"
        self.local_costmap_info = "local costmap: waiting"
        self.trail = deque(maxlen=max(2, self.trail_max_points))

        self.marker_pub = self.create_publisher(MarkerArray, "/viz/robot_markers", 10)
        self.trail_pub = self.create_publisher(Path, "/viz/robot_trail", 10)
        self.plan_pub = self.create_publisher(Path, "/viz/global_plan", 10)

        self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.create_subscription(Twist, self.raw_cmd_topic, self.raw_cmd_cb, 10)
        self.create_subscription(Twist, self.adjusted_cmd_topic, self.adjusted_cmd_cb, 10)
        self.create_subscription(Path, self.plan_topic, self.plan_cb, 10)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.global_costmap_cb, 10)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.local_costmap_cb, 10)

        hz = float(self.get_parameter("publish_frequency_hz").value)
        self.timer = self.create_timer(1.0 / max(hz, 1.0), self.publish_visuals)

        self.get_logger().info(
            "RViz visualizer ready: markers=/viz/robot_markers, "
            "trail=/viz/robot_trail, enhanced_plan=/viz/global_plan")

    def pose_cb(self, msg: PoseStamped):
        self.current_pose = msg
        self.current_yaw = quat_to_yaw(msg.pose.orientation)

        if not self.trail:
            self.trail.append(msg)
            return

        last = self.trail[-1].pose.position
        cur = msg.pose.position
        dist = math.hypot(cur.x - last.x, cur.y - last.y)
        if dist >= self.trail_min_distance:
            self.trail.append(msg)

    def raw_cmd_cb(self, msg: Twist):
        self.raw_cmd = msg

    def adjusted_cmd_cb(self, msg: Twist):
        self.adjusted_cmd = msg

    def plan_cb(self, msg: Path):
        self.global_plan = msg
        out = Path()
        out.header = msg.header
        out.header.frame_id = self.frame_id
        out.poses = msg.poses
        for pose in out.poses:
            pose.header.frame_id = self.frame_id
        self.plan_pub.publish(out)

    def global_costmap_cb(self, msg: OccupancyGrid):
        self.global_costmap_info = self._costmap_summary("global", msg)

    def local_costmap_cb(self, msg: OccupancyGrid):
        self.local_costmap_info = self._costmap_summary("local", msg)

    def publish_visuals(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        if self.current_pose is None:
            markers.markers.append(self._status_text_marker(now, None))
            self.marker_pub.publish(markers)
            return

        pose = self.current_pose.pose
        markers.markers.extend([
            self._robot_body_marker(now, pose),
            self._heading_marker(now, pose),
            self._velocity_marker(now, pose, self.raw_cmd, 10, "raw_cmd", (0.20, 0.45, 1.0)),
            self._velocity_marker(now, pose, self.adjusted_cmd, 11, "adjusted_cmd", (0.05, 0.80, 0.35)),
            self._status_text_marker(now, pose),
        ])

        if self.global_plan.poses:
            markers.markers.append(self._plan_marker(now))

        self.marker_pub.publish(markers)
        self.trail_pub.publish(self._trail_path(now))

    def _robot_body_marker(self, stamp, pose):
        marker = self._base_marker(stamp, 1, "robot")
        marker.type = Marker.CYLINDER
        marker.pose = pose
        marker.scale.x = self.robot_radius * 2.0
        marker.scale.y = self.robot_radius * 2.0
        marker.scale.z = 0.10
        marker.color.r = 0.95
        marker.color.g = 0.55
        marker.color.b = 0.10
        marker.color.a = 0.75
        return marker

    def _heading_marker(self, stamp, pose):
        marker = self._base_marker(stamp, 2, "robot")
        marker.type = Marker.ARROW
        marker.points = [
            self._point(pose.position.x, pose.position.y, 0.15),
            self._point(
                pose.position.x + self.heading_arrow_length * math.cos(self.current_yaw),
                pose.position.y + self.heading_arrow_length * math.sin(self.current_yaw),
                0.15,
            ),
        ]
        marker.scale.x = 0.06
        marker.scale.y = 0.14
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.20
        marker.color.b = 0.05
        marker.color.a = 1.0
        return marker

    def _velocity_marker(self, stamp, pose, cmd, marker_id, namespace, color):
        marker = self._base_marker(stamp, marker_id, namespace)
        marker.type = Marker.ARROW
        vx = float(cmd.linear.x)
        vy = float(cmd.linear.y)
        wx = vx * math.cos(self.current_yaw) - vy * math.sin(self.current_yaw)
        wy = vx * math.sin(self.current_yaw) + vy * math.cos(self.current_yaw)
        marker.points = [
            self._point(pose.position.x, pose.position.y, 0.28),
            self._point(
                pose.position.x + wx * self.velocity_arrow_scale,
                pose.position.y + wy * self.velocity_arrow_scale,
                0.28,
            ),
        ]
        marker.scale.x = 0.035
        marker.scale.y = 0.09
        marker.scale.z = 0.12
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.95
        return marker

    def _status_text_marker(self, stamp, pose):
        marker = self._base_marker(stamp, 20, "status")
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        if pose is None:
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.6
            marker.text = "MPPI RViz: waiting for pose"
            return marker

        marker.pose.position.x = pose.position.x
        marker.pose.position.y = pose.position.y + 0.65
        marker.pose.position.z = 0.55
        raw_speed = math.hypot(self.raw_cmd.linear.x, self.raw_cmd.linear.y)
        adj_speed = math.hypot(self.adjusted_cmd.linear.x, self.adjusted_cmd.linear.y)
        marker.text = (
            f"pose=({pose.position.x:.2f}, {pose.position.y:.2f}) yaw={self.current_yaw:.2f} rad\n"
            f"raw vx={self.raw_cmd.linear.x:.2f} vy={self.raw_cmd.linear.y:.2f} "
            f"wz={self.raw_cmd.angular.z:.2f} speed={raw_speed:.2f}\n"
            f"adj vx={self.adjusted_cmd.linear.x:.2f} vy={self.adjusted_cmd.linear.y:.2f} "
            f"wz={self.adjusted_cmd.angular.z:.2f} speed={adj_speed:.2f}\n"
            f"{self.global_costmap_info} | {self.local_costmap_info}"
        )
        return marker

    def _plan_marker(self, stamp):
        marker = self._base_marker(stamp, 30, "global_plan")
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.045
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 0.90
        marker.points = [
            self._point(ps.pose.position.x, ps.pose.position.y, 0.08)
            for ps in self.global_plan.poses
        ]
        return marker

    def _trail_path(self, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self.frame_id
        path.poses = list(self.trail)
        for pose in path.poses:
            pose.header.frame_id = self.frame_id
        return path

    def _base_marker(self, stamp, marker_id: int, namespace: str):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    @staticmethod
    def _point(x: float, y: float, z: float):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def _costmap_summary(name: str, msg: OccupancyGrid):
        occupied = sum(1 for value in msg.data if value > 0)
        return f"{name} costmap: {msg.info.width}x{msg.info.height}, occ={occupied}"


def main(args=None):
    rclpy.init(args=args)
    node = MppiRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
