"""CLI: 发 GoToPose 目标 — 只需要 x, y, yaw

用法:
  nav2_goal 7.0 2.0 90                # yaw 用度
  nav2_goal 7.0 2.0 1.57 --rad        # yaw 用弧度
  nav2_goal 7.0 2.0 90 --timeout 30   # 30 秒超时
"""

import math
import sys
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_pose_navigator_interfaces.action import GoToPose
from geometry_msgs.msg import PoseStamped


def yaw_to_quat(yaw: float):
    """yaw (rad) → geometry_msgs/Quaternion"""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class GoalSender(Node):
    def __init__(self):
        super().__init__("nav2_goal")
        self._client = ActionClient(self, GoToPose, "/go_to_pose")
        self._done = False
        self._success = False
        self._msg = ""

        while not self._client.wait_for_server(timeout_sec=2.0):
            if not rclpy.ok():
                return
            self.get_logger().info("等待 /go_to_pose Action 服务器...")

    def send(self, x: float, y: float, yaw: float, timeout_s: float):
        goal = GoToPose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = yaw_to_quat(yaw)
        goal.timeout_sec = timeout_s

        self.get_logger().info(
            f"目标: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.0f}°"
            + (f", 超时={timeout_s:.0f}s" if timeout_s > 0 else ""))
        self._client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("目标被拒绝")
            self._done = True
            return
        self.get_logger().info("目标已接受，开始导航...")
        goal_handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        self._success = result.success
        self._msg = result.message
        self._done = True

    @property
    def done(self):
        return self._done

    @property
    def success(self):
        return self._success

    @property
    def message(self):
        return self._msg


def main(args=None):
    # 用 argparse 解析 CLI（绕过 rclpy 的参数解析冲突）
    parser = argparse.ArgumentParser(description="发送 GoToPose 导航目标")
    parser.add_argument("x", type=float, help="目标 X (map 坐标系)")
    parser.add_argument("y", type=float, help="目标 Y (map 坐标系)")
    parser.add_argument("yaw", type=float, help="目标朝向")
    parser.add_argument("--rad", action="store_true", help="yaw 用弧度 (默认: 度)")
    parser.add_argument("--timeout", type=float, default=60.0, help="超时秒数 (默认: 60)")
    args, ros_args = parser.parse_known_args()

    yaw = args.yaw if args.rad else math.radians(args.yaw)

    rclpy.init(args=[sys.argv[0]] + ros_args)
    node = GoalSender()

    if not rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.send(args.x, args.y, yaw, args.timeout)

    import time
    start = time.time()
    while rclpy.ok() and not node.done and (time.time() - start < args.timeout):
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.done:
        status = "✅ 成功" if node.success else "❌ 失败"
        print(f"\n{status}: {node.message}")
    else:
        print(f"\n⏰ 超时 ({args.timeout}s)")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node.success else 1)


if __name__ == "__main__":
    main()
