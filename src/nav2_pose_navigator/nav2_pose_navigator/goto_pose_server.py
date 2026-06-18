"""GoToPose Action server — wraps Nav2 NavigateToPose with progress feedback."""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.client import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from nav2_pose_navigator_interfaces.action import GoToPose


class GoToPoseServer(Node):
    def __init__(self):
        super().__init__("goto_pose_server")

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

        self.get_logger().info("GoToPoseServer ready — waiting for goals on /go_to_pose")

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

        if not self.nav2_client.wait_for_server(timeout_sec=5.0):
            goal_handle.abort()
            return GoToPose.Result(success=False, message="Nav2 action server not available")

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose

        self.get_logger().info("Sending to Nav2 …")
        send_future = self.nav2_client.send_goal_async(nav2_goal)

        while not send_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return GoToPose.Result(success=False, message="Cancelled")
            await self._sleep(0.05)

        goal_handle_nav2 = send_future.result()
        if goal_handle_nav2 is None or not goal_handle_nav2.accepted:
            goal_handle.abort()
            return GoToPose.Result(success=False, message="Nav2 rejected goal")

        self._active_nav2_handle = goal_handle_nav2
        result_future = goal_handle_nav2.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle_nav2.cancel_goal_async()
                goal_handle.canceled()
                self._active_nav2_handle = None
                return GoToPose.Result(success=False, message="Cancelled")

            dist = self._compute_distance(target_pose)
            goal_handle.publish_feedback(GoToPose.Feedback(distance_remaining=dist))
            await self._sleep(0.1)

        self._active_nav2_handle = None
        nav2_result = result_future.result()

        result = GoToPose.Result()
        if nav2_result.status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.message = "Arrived at target"
        else:
            result.success = False
            result.message = f"Nav2 failed with status {nav2_result.status}"

        self.get_logger().info(result.message)
        return result

    def pose_cb(self, msg: PoseStamped):
        self._current_pose = msg

    def _compute_distance(self, target: PoseStamped) -> float:
        if self._current_pose is None:
            return float("inf")
        dx = self._current_pose.pose.position.x - target.pose.position.x
        dy = self._current_pose.pose.position.y - target.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    async def _sleep(self, seconds: float):
        await rclpy.task.Future()._asyncio_future_blocking_loop(
            lambda: None, timeout_sec=seconds
        )


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
