"""GoToPose Action server — wraps Nav2 NavigateToPose with progress feedback."""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from nav2_pose_navigator_interfaces.action import GoToPose


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__("goto_pose_server")

        self.declare_parameter("team", "red")
        self.declare_parameter("ramp_x_min", 8.9)
        self.declare_parameter("pre_ramp_offset", 0.25)
        self.team = self.get_parameter("team").value
        self.ramp_x_min = self.get_parameter("ramp_x_min").value
        self.pre_ramp_offset = self.get_parameter("pre_ramp_offset").value

        # ramp Y center: align to middle of the ramp before climbing
        if self.team == "blue":
            ramp_y_min, ramp_y_max = 3.1, 4.6
        else:
            ramp_y_min, ramp_y_max = -4.6, -3.1
        self.ramp_y_center = (ramp_y_min + ramp_y_max) / 2.0

        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self._action_server = ActionServer(
            self,
            GoToPose,
            "go_to_pose",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self._current_pose = None
        self._active_nav2_handle = None
        self.pose_sub = self.create_subscription(
            PoseStamped, "/odin1/relocation", self.pose_cb, 10
        )

        self.get_logger().info(
            f"GoToPoseServer ready — waiting for goals on /go_to_pose "
            f"(team={self.team}, ramp_x_min={self.ramp_x_min}, "
            f"ramp_y_center={self.ramp_y_center:.2f}, pre_ramp_offset={self.pre_ramp_offset})")

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal: ({goal_request.target_pose.pose.position.x:.2f}, "
            f"{goal_request.target_pose.pose.position.y:.2f})"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        if self._active_nav2_handle is not None:
            self._active_nav2_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        target_pose = goal_handle.request.target_pose
        timeout_sec = goal_handle.request.timeout_sec
        tx = target_pose.pose.position.x
        ty = target_pose.pose.position.y

        if timeout_sec > 0:
            self.get_logger().info(f"Timeout set: {timeout_sec:.1f}s")
        start_time = self.get_clock().now()

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            return GoToPose.Result(success=False, message="Nav2 action server not available")

        # ── Pre-ramp alignment: if goal is beyond ramp, align first ──
        if tx >= self.ramp_x_min:
            pre_result = await self._navigate_to_pre_ramp(
                goal_handle, target_pose, tx, ty,
                start_time=start_time, timeout_sec=timeout_sec)
            if pre_result is not None:
                return pre_result  # failed or cancelled

        # ── Main navigation to final target ──
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose

        self.get_logger().info(f"Sending final goal: ({tx:.2f}, {ty:.2f})")
        result = await self._run_navigate(
            goal_handle, nav2_goal, target_pose,
            start_time=start_time, timeout_sec=timeout_sec)
        if result is None:
            result = GoToPose.Result(success=True, message="Goal reached")
        self.get_logger().info(result.message)
        return result

    async def _navigate_to_pre_ramp(self, goal_handle, target_pose, tx, ty,
                                      start_time=None, timeout_sec=0.0):
        """Navigate to pre-ramp alignment point (0.25m before ramp, face +X).
        Aligns to ramp Y center, regardless of final goal Y."""
        pre_x = self.ramp_x_min - self.pre_ramp_offset  # e.g. 8.4
        pre_y = self.ramp_y_center

        pre_pose = PoseStamped()
        pre_pose.header.frame_id = "map"
        pre_pose.header.stamp = target_pose.header.stamp
        pre_pose.pose.position.x = pre_x
        pre_pose.pose.position.y = pre_y
        pre_pose.pose.position.z = 0.0
        # yaw=0: face straight up the ramp (+X)
        pre_pose.pose.orientation.z = 0.0
        pre_pose.pose.orientation.w = 1.0

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = pre_pose

        self.get_logger().info(
            f"Phase 1 — align before ramp: ({pre_x:.2f}, {pre_y:.2f}, yaw=0)")
        return await self._run_navigate(
            goal_handle, nav2_goal, pre_pose, label="pre-ramp",
            start_time=start_time, timeout_sec=timeout_sec)

    async def _run_navigate(self, goal_handle, nav2_goal, ref_pose,
                             label="main", start_time=None, timeout_sec=0.0):
        """Run a single NavigateToPose and return GoToPose.Result on
        failure/cancel/timeout, or None on success (caller continues)."""
        send_future = self.nav2_client.send_goal_async(nav2_goal)

        while not send_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return GoToPose.Result(success=False, message="Cancelled")
            if self._check_timeout(start_time, timeout_sec):
                return GoToPose.Result(
                    success=False,
                    message=f"Timeout {timeout_sec:.0f}s during {label} send")
            await self._sleep(0.05)

        goal_handle_nav2 = send_future.result()
        if goal_handle_nav2 is None or not goal_handle_nav2.accepted:
            return GoToPose.Result(success=False, message=f"Nav2 rejected {label} goal")

        self._active_nav2_handle = goal_handle_nav2
        result_future = goal_handle_nav2.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle_nav2.cancel_goal_async()
                goal_handle.canceled()
                self._active_nav2_handle = None
                return GoToPose.Result(success=False, message="Cancelled")

            if self._check_timeout(start_time, timeout_sec):
                goal_handle_nav2.cancel_goal_async()
                self._active_nav2_handle = None
                return GoToPose.Result(
                    success=False,
                    message=f"Timeout {timeout_sec:.0f}s during {label}")

            dist = self._compute_distance(ref_pose)
            goal_handle.publish_feedback(
                GoToPose.Feedback(distance_remaining=dist))
            await self._sleep(0.1)

        self._active_nav2_handle = None
        nav2_result = result_future.result()

        if nav2_result.status != GoalStatus.STATUS_SUCCEEDED:
            return GoToPose.Result(
                success=False,
                message=f"Nav2 {label} failed with status {nav2_result.status}")
        return None  # success, continue

    def pose_cb(self, msg: PoseStamped):
        self._current_pose = msg

    def _compute_distance(self, target: PoseStamped) -> float:
        if self._current_pose is None:
            return float("inf")
        dx = self._current_pose.pose.position.x - target.pose.position.x
        dy = self._current_pose.pose.position.y - target.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def _check_timeout(self, start_time, timeout_sec: float) -> bool:
        """Return True if timeout exceeded. 0 or negative = no timeout."""
        if timeout_sec <= 0 or start_time is None:
            return False
        elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
        return elapsed > timeout_sec

    async def _sleep(self, seconds: float):
        """Sleep using rclpy Future (no asyncio event loop needed)."""
        future = Future()
        timer = self.create_timer(seconds, lambda: future.set_result(None))
        await future
        self.destroy_timer(timer)


def main(args=None):
    rclpy.init(args=args)
    node = GoToPoseServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
