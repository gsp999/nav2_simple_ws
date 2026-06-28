"""Simple local robot simulator for Nav2 closed-loop testing.

It integrates the adjusted velocity command into a planar pose, then publishes
the same Odin-facing pose and odometry topics used by the real robot.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quat(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Nav2SimRobot(Node):
    def __init__(self):
        super().__init__("nav2_sim_robot")

        self.declare_parameter("cmd_topic", "/cmd_vel_adjusted")
        self.declare_parameter("pose_topic", "/odin1/relocation")
        self.declare_parameter("odom_topic", "/odin1/odometry_highfreq")
        self.declare_parameter("frame_id", "nav_map")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_frequency_hz", 50.0)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("initial_x", 0.5)
        self.declare_parameter("initial_y", -3.8)
        self.declare_parameter("initial_yaw", 0.0)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.publish_frequency_hz = float(
            self.get_parameter("publish_frequency_hz").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_update_time = self.last_cmd_time

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, self.cmd_topic, self.cmd_cb, 10)

        period = 1.0 / max(self.publish_frequency_hz, 1.0)
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            "nav2_sim_robot: "
            f"cmd={self.cmd_topic}, pose={self.pose_topic}, "
            f"odom={self.odom_topic}, initial=({self.x:.2f}, "
            f"{self.y:.2f}, yaw={self.yaw:.2f})")

    def cmd_cb(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0:
            return

        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        active = cmd_age <= self.command_timeout_sec
        vx = float(self.cmd.linear.x) if active else 0.0
        vy = float(self.cmd.linear.y) if active else 0.0
        wz = float(self.cmd.angular.z) if active else 0.0

        world_vx = vx * math.cos(self.yaw) - vy * math.sin(self.yaw)
        world_vy = vx * math.sin(self.yaw) + vy * math.cos(self.yaw)
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw = normalize_angle(self.yaw + wz * dt)

        self.publish_state(now, vx, vy, wz)

    def publish_state(self, now, vx: float, vy: float, wz: float):
        qx, qy, qz, qw = yaw_to_quat(self.yaw)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        odom = Odometry()
        odom.header.stamp = pose.header.stamp
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
