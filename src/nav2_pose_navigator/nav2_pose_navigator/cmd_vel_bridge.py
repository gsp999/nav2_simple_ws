"""Twist (/cmd_vel) -> smoothed Float32MultiArray (/t0x0101_action).

The hardware command topic expects [vx, vy, wz].  Nav2 controllers can change
their command sharply from one cycle to the next, so this bridge applies a
simple acceleration limit before forwarding commands to the chassis.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")

        self.declare_parameter("publish_frequency_hz", 100.0)
        self.declare_parameter("command_timeout_sec", 0.3)
        self.declare_parameter("max_linear_accel", 1.0)
        self.declare_parameter("max_angular_accel", 2.0)

        self.publish_frequency_hz = float(
            self.get_parameter("publish_frequency_hz").value)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value)
        self.max_linear_accel = float(
            self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(
            self.get_parameter("max_angular_accel").value)

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_publish_time = self.last_cmd_time

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/t0x0101_action", 10)
        period = 1.0 / max(self.publish_frequency_hz, 1.0)
        self.timer = self.create_timer(period, self.publish_smoothed)
        self.get_logger().info(
            "cmd_vel_bridge: /cmd_vel -> /t0x0101_action [vx, vy, wz], "
            f"rate={self.publish_frequency_hz:.1f}Hz, "
            f"lin_acc={self.max_linear_accel:.2f}m/s^2, "
            f"ang_acc={self.max_angular_accel:.2f}rad/s^2")

    def cb(self, msg: Twist):
        self.target_vx = float(msg.linear.x)
        self.target_vy = float(msg.linear.y)
        self.target_wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()

    def publish_smoothed(self):
        now = self.get_clock().now()
        dt = (now - self.last_publish_time).nanoseconds / 1e9
        self.last_publish_time = now
        if dt <= 0.0:
            return

        if self.command_timeout_sec > 0.0:
            cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
            if cmd_age > self.command_timeout_sec:
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_wz = 0.0

        self.current_vx = self._step_scalar(
            self.current_vx, self.target_vx, self.max_linear_accel * dt)
        self.current_vy = self._step_scalar(
            self.current_vy, self.target_vy, self.max_linear_accel * dt)
        self.current_wz = self._step_scalar(
            self.current_wz, self.target_wz, self.max_angular_accel * dt)

        out = Float32MultiArray()
        out.data = [self.current_vx, self.current_vy, self.current_wz]
        self.pub.publish(out)

    @staticmethod
    def _step_scalar(current: float, target: float, max_step: float) -> float:
        if not math.isfinite(target):
            target = 0.0
        max_step = abs(max_step)
        delta = target - current
        if abs(delta) <= max_step:
            return target
        return current + math.copysign(max_step, delta)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
