"""Manual lifecycle bringup — replaces nav2_lifecycle_manager on slow hardware.

Waits for lifecycle services with short polling, then performs sequential
configure→activate transitions with retries. This keeps the Radxa-friendly
sequential behavior without paying a fixed long startup delay every run.
"""

import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


class LifecycleBringup(Node):
    def __init__(self):
        super().__init__("lifecycle_bringup")

        self.declare_parameter("node_names", [""])
        self.declare_parameter("sleep_configure", 2.0)
        self.declare_parameter("sleep_activate", 1.0)
        self.declare_parameter("service_timeout", 5.0)
        self.declare_parameter("service_poll_period", 0.25)
        self.declare_parameter("transition_timeout", 10.0)
        self.declare_parameter("transition_retries", 2)
        self.declare_parameter("retry_delay", 0.5)

        node_names = self.get_parameter("node_names").get_parameter_value().string_array_value
        sleep_cfg = float(self.get_parameter("sleep_configure").value)
        sleep_act = float(self.get_parameter("sleep_activate").value)
        svc_timeout = float(self.get_parameter("service_timeout").value)
        poll_period = float(self.get_parameter("service_poll_period").value)
        transition_timeout = float(
            self.get_parameter("transition_timeout").value)
        transition_retries = int(
            self.get_parameter("transition_retries").value)
        retry_delay = float(self.get_parameter("retry_delay").value)

        if not node_names or node_names == [""]:
            self.get_logger().error("node_names parameter is empty, nothing to do.")
            self._done(False)
            return

        self.get_logger().info(
            f"Bringing up {len(node_names)} nodes: {node_names}. "
            f"service_timeout={svc_timeout:.1f}s, "
            f"transition_timeout={transition_timeout:.1f}s"
        )

        clients = self._wait_for_services(node_names, svc_timeout, poll_period)
        if clients is None:
            self._done(False)
            return

        # Phase 1: CONFIGURE
        self.get_logger().info("=== Phase 1: CONFIGURE ===")
        for name in node_names:
            if not self._change_state(
                clients[name],
                name,
                Transition.TRANSITION_CONFIGURE,
                "configured",
                transition_timeout,
                transition_retries,
                retry_delay,
            ):
                self._done(False)
                return
            time.sleep(sleep_cfg)

        # Phase 2: ACTIVATE
        self.get_logger().info("=== Phase 2: ACTIVATE ===")
        for name in node_names:
            if not self._change_state(
                clients[name],
                name,
                Transition.TRANSITION_ACTIVATE,
                "activated",
                transition_timeout,
                transition_retries,
                retry_delay,
            ):
                self._done(False)
                return
            time.sleep(sleep_act)

        self.get_logger().info("=== All Nav2 nodes active! ===")
        self._done(True)

    def _done(self, success):
        self.get_logger().info("Lifecycle bringup complete, shutting down.")
        # Short delay so logs flush
        time.sleep(0.1)
        raise SystemExit(0 if success else 1)

    def _wait_for_services(self, node_names, max_wait, poll_period):
        clients = {
            name: self.create_client(ChangeState, f"/{name}/change_state")
            for name in node_names
        }
        pending = set(node_names)
        deadline = time.monotonic() + max_wait

        while pending and time.monotonic() < deadline:
            for name in list(pending):
                if clients[name].wait_for_service(timeout_sec=0.0):
                    pending.remove(name)
                    self.get_logger().info(f"  Found /{name}/change_state")
            if pending:
                time.sleep(poll_period)

        if pending:
            self.get_logger().error(
                f"{len(pending)} lifecycle service(s) not available after "
                f"{max_wait:.1f}s: {[f'/{n}/change_state' for n in sorted(pending)]}"
            )
            return None
        return clients

    def _change_state(
        self,
        client,
        name,
        transition_id,
        success_word,
        timeout,
        retries,
        retry_delay,
    ):
        action = success_word.replace("ed", "ing")
        for attempt in range(1, retries + 2):
            self.get_logger().info(
                f"{action.capitalize()} {name} "
                f"(attempt {attempt}/{retries + 1})...")
            req = ChangeState.Request()
            req.transition = Transition(id=transition_id)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if future.done() and future.result() and future.result().success:
                self.get_logger().info(f"  {name} {success_word} OK")
                return True

            if attempt <= retries:
                self.get_logger().warn(
                    f"  {name} {success_word} not ready, retrying in "
                    f"{retry_delay:.1f}s")
                time.sleep(retry_delay)

        self.get_logger().error(f"  {name} {success_word} FAILED")
        return False


def main():
    rclpy.init()
    try:
        node = LifecycleBringup()
        # rclpy.spin() would block; the node exits via _done
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(f"lifecycle_bringup failed: {e}")
    finally:
        rclpy.shutdown()
