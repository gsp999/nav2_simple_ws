"""监测位置，坡面抬升悬挂 + 可选速度/yaw 兜底控制

进入坡面时自动:
  1. 升起悬挂 (suspension_ramp)
  2. 可选锁 yaw=0 (正面朝坡，+X 方向)
  3. 可选强制坡面最低速 (min_ramp_speed)
  4. 可选发布减速模式 (/targetstate=-1)

出坡面时恢复:
  1. 降悬挂 (suspension_flat)
  2. 如果启用了坡面 yaw 锁，则释放 yaw 锁
  3. 可选发布减速模式 (/targetstate=-1)

外部 yaw 覆盖:
  - /desired_yaw (Float32): 设置期望朝向(rad)，发 999.0 解除
  - 外部覆盖优先级高于坡面自动锁（可在坡面中覆盖）
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, Float32MultiArray, Float32, Int32


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RampZoneManager(Node):
    def __init__(self):
        super().__init__("ramp_zone_manager")
        self.declare_parameter("team", "red")
        self.declare_parameter("ramp_x_min", 8.9)
        self.declare_parameter("ramp_x_max", 10.4)
        self.declare_parameter("min_ramp_speed", 0.25)
        self.declare_parameter("suspension_ramp", 75.0)
        self.declare_parameter("suspension_flat", 30.0)
        self.declare_parameter("yaw_kp", 2.0)
        self.declare_parameter("yaw_max_vel", 2.0)
        self.declare_parameter("enable_ramp_yaw_lock", False)
        self.declare_parameter("enable_ramp_speed_floor", False)
        self.declare_parameter("publish_ramp_target_state", False)
        self.declare_parameter("enable_cruise_speed_floor", True)
        self.declare_parameter("min_cruise_speed", 1.50)
        self.declare_parameter("enable_cruise_angular_floor", True)
        self.declare_parameter("min_cruise_angular_speed", 0.45)
        self.declare_parameter("cruise_slowdown_distance", 1.0)
        self.declare_parameter("cruise_command_epsilon", 0.02)
        self.declare_parameter("cruise_angular_epsilon", 0.03)

        self.team = self.get_parameter("team").value
        self.ramp_x_min = self.get_parameter("ramp_x_min").value
        self.ramp_x_max = self.get_parameter("ramp_x_max").value
        self.min_ramp_speed = self.get_parameter("min_ramp_speed").value
        self.suspension_ramp = self.get_parameter("suspension_ramp").value
        self.suspension_flat = self.get_parameter("suspension_flat").value
        self.yaw_kp = self.get_parameter("yaw_kp").value
        self.yaw_max_vel = self.get_parameter("yaw_max_vel").value
        self.enable_ramp_yaw_lock = bool(
            self.get_parameter("enable_ramp_yaw_lock").value)
        self.enable_ramp_speed_floor = bool(
            self.get_parameter("enable_ramp_speed_floor").value)
        self.publish_ramp_target_state = bool(
            self.get_parameter("publish_ramp_target_state").value)
        self.enable_cruise_speed_floor = bool(
            self.get_parameter("enable_cruise_speed_floor").value)
        self.min_cruise_speed = float(
            self.get_parameter("min_cruise_speed").value)
        self.enable_cruise_angular_floor = bool(
            self.get_parameter("enable_cruise_angular_floor").value)
        self.min_cruise_angular_speed = float(
            self.get_parameter("min_cruise_angular_speed").value)
        self.cruise_slowdown_distance = float(
            self.get_parameter("cruise_slowdown_distance").value)
        self.cruise_command_epsilon = float(
            self.get_parameter("cruise_command_epsilon").value)
        self.cruise_angular_epsilon = float(
            self.get_parameter("cruise_angular_epsilon").value)

        if self.team == "blue":
            self.ramp_y_min, self.ramp_y_max = 3.1, 4.6
        else:
            self.ramp_y_min, self.ramp_y_max = -4.6, -3.1

        self.in_ramp = False
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.desired_yaw = None  # None = 不覆盖，让 Nav2 控制
        self.navigation_active = False
        self.distance_remaining = float("inf")

        # 订阅
        self.pose_sub = self.create_subscription(
            PoseStamped, "/odin1/relocation", self.pose_cb, 10)
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_cb, 10)
        self.yaw_sub = self.create_subscription(
            Float32, "/desired_yaw", self.yaw_target_cb, 10)
        self.nav_active_sub = self.create_subscription(
            Bool, "/go_to_pose/active", self.nav_active_cb, 10)
        self.distance_sub = self.create_subscription(
            Float32, "/go_to_pose/distance_remaining", self.distance_cb, 10)

        # 发布
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_adjusted", 10)
        self.suspension_pub = self.create_publisher(
            Float32MultiArray, "/t0x0112_action", 10)
        self.target_state_pub = self.create_publisher(Int32, "/targetstate", 10)

        self.get_logger().info(
            f"RampZoneManager [{self.team}]: yaw_kp={self.yaw_kp}, "
            f"ramp y=[{self.ramp_y_min},{self.ramp_y_max}], "
            f"cruise_floor={self.enable_cruise_speed_floor}, "
            f"min_cruise={self.min_cruise_speed:.2f}m/s, "
            f"angular_floor={self.enable_cruise_angular_floor}, "
            f"min_wz={self.min_cruise_angular_speed:.2f}rad/s, "
            f"ramp_yaw_lock={self.enable_ramp_yaw_lock}, "
            f"ramp_speed_floor={self.enable_ramp_speed_floor}")

    def nav_active_cb(self, msg: Bool):
        self.navigation_active = bool(msg.data)
        if not self.navigation_active:
            self.distance_remaining = float("inf")

    def distance_cb(self, msg: Float32):
        self.distance_remaining = float(msg.data)

    def yaw_target_cb(self, msg: Float32):
        """设置期望 yaw（rad）。发 NaN 或 >900 禁用覆盖"""
        if math.isnan(msg.data) or msg.data > 900:
            if self.desired_yaw is not None:
                self.get_logger().info("Yaw override disabled")
            self.desired_yaw = None
        else:
            self.desired_yaw = msg.data
            self.get_logger().info(
                f"Yaw override: {math.degrees(self.desired_yaw):.0f} deg")

    def pose_cb(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_yaw = quat_to_yaw(msg.pose.orientation)

        was_in_ramp = self.in_ramp
        self.in_ramp = (
            self.ramp_x_min <= self.current_x <= self.ramp_x_max and
            self.ramp_y_min <= self.current_y <= self.ramp_y_max
        )
        if self.in_ramp and not was_in_ramp:
            self.get_logger().info("Enter ramp -> raise suspension")
            h = self.suspension_ramp
            self.suspension_pub.publish(Float32MultiArray(data=[h, h, h, h]))
            if self.publish_ramp_target_state:
                self.target_state_pub.publish(Int32(data=-1))
            if self.enable_ramp_yaw_lock:
                # Lock yaw to 0 (straight up +X), face the ramp head-on
                self.desired_yaw = 0.0
                self.get_logger().info("Yaw locked to 0° (face ramp)")
        elif not self.in_ramp and was_in_ramp:
            self.get_logger().info("Exit ramp -> lower suspension")
            h = self.suspension_flat
            self.suspension_pub.publish(Float32MultiArray(data=[h, h, h, h]))
            if self.publish_ramp_target_state:
                self.target_state_pub.publish(Int32(data=-1))
            if self.enable_ramp_yaw_lock:
                # Release yaw — Nav2 resumes orientation control (rotate to goal)
                self.desired_yaw = None
                self.get_logger().info("Yaw unlocked (Nav2 resumes rotation)")

    def cmd_cb(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.angular.z = msg.angular.z

        # Yaw 覆盖
        if self.desired_yaw is not None:
            yaw_error = normalize_angle(self.desired_yaw - self.current_yaw)
            wz = self.yaw_kp * yaw_error
            wz = max(-self.yaw_max_vel, min(self.yaw_max_vel, wz))
            out.angular.z = wz

        # Ramp 最低速（默认关闭：坡面只抬底盘，线速度交给巡航兜底/Nav2）
        if self.in_ramp and self.enable_ramp_speed_floor:
            speed = math.sqrt(out.linear.x**2 + out.linear.y**2)
            if 0.01 < speed < self.min_ramp_speed:
                scale = self.min_ramp_speed / speed
                out.linear.x *= scale
                out.linear.y *= scale
        elif self._should_apply_linear_floor():
            speed = math.sqrt(out.linear.x**2 + out.linear.y**2)
            if self.cruise_command_epsilon < speed < self.min_cruise_speed:
                scale = self.min_cruise_speed / speed
                out.linear.x *= scale
                out.linear.y *= scale

        # 角速度兜底不能按距离关闭：Nav2 通常接近目标后才开始调整最终朝向。
        if self._should_apply_angular_floor():
            wz = abs(out.angular.z)
            if self.cruise_angular_epsilon < wz < self.min_cruise_angular_speed:
                out.angular.z = math.copysign(
                    self.min_cruise_angular_speed, out.angular.z)

        self.cmd_pub.publish(out)

    def _should_apply_linear_floor(self) -> bool:
        return (
            self.enable_cruise_speed_floor and
            self.navigation_active and
            self.distance_remaining > self.cruise_slowdown_distance
        )

    def _should_apply_angular_floor(self) -> bool:
        return (
            self.enable_cruise_angular_floor and
            self.navigation_active and
            self.desired_yaw is None
        )


def main(args=None):
    rclpy.init(args=args)
    node = RampZoneManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
