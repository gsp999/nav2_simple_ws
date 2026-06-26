"""Manual lifecycle bringup — replaces nav2_lifecycle_manager on slow hardware.

Does sequential configure→activate with generous sleep between each step,
avoiding DDS timeouts that plague ARM boards (Radxa Airbox Q900).
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

        node_names = self.get_parameter("node_names").get_parameter_value().string_array_value
        sleep_cfg = self.get_parameter("sleep_configure").value
        sleep_act = self.get_parameter("sleep_activate").value
        svc_timeout = self.get_parameter("service_timeout").value

        if not node_names or node_names == [""]:
            self.get_logger().error("node_names parameter is empty, nothing to do.")
            self._done(False)
            return

        self.get_logger().info(
            f"Bringing up {len(node_names)} nodes: {node_names}"
        )

        # Build service clients — with retries for slow ARM boards where
        # costmap nodes (inside planner/controller) take a long time to
        # finish constructing and expose their /change_state service.
        clients = {}
        pending = list(node_names)
        max_attempts = 5
        for attempt in range(max_attempts):
            still_pending = []
            for name in pending:
                srv_name = f"/{name}/change_state"
                cli = clients.get(name)
                if cli is None:
                    cli = self.create_client(ChangeState, srv_name)
                if cli.wait_for_service(timeout_sec=svc_timeout):
                    clients[name] = cli
                    self.get_logger().info(f"  Found {srv_name}")
                else:
                    still_pending.append(name)

            pending = still_pending
            if not pending:
                break

            if attempt < max_attempts - 1:
                wait = 3.0 * (attempt + 1)
                self.get_logger().warn(
                    f"{len(pending)} service(s) not ready yet: {pending}. "
                    f"Retrying in {wait:.0f}s (attempt {attempt+2}/{max_attempts})..."
                )
                time.sleep(wait)

        if pending:
            self.get_logger().error(
                f"{len(pending)} service(s) still not available after "
                f"{max_attempts} attempts: {[f'/{n}/change_state' for n in pending]}"
            )
            self._done(False)
            return

        # Phase 1: CONFIGURE
        self.get_logger().info("=== Phase 1: CONFIGURE ===")
        for name in node_names:
            self.get_logger().info(f"Configuring {name}...")
            req = ChangeState.Request()
            req.transition = Transition(id=Transition.TRANSITION_CONFIGURE)
            future = clients[name].call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=svc_timeout)
            if future.done() and future.result() and future.result().success:
                self.get_logger().info(f"  {name} configured OK")
            else:
                self.get_logger().error(f"  {name} configure FAILED")
                self._done(False)
                return
            time.sleep(sleep_cfg)

        # Phase 2: ACTIVATE
        self.get_logger().info("=== Phase 2: ACTIVATE ===")
        for name in node_names:
            self.get_logger().info(f"Activating {name}...")
            req = ChangeState.Request()
            req.transition = Transition(id=Transition.TRANSITION_ACTIVATE)
            future = clients[name].call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=svc_timeout)
            if future.done() and future.result() and future.result().success:
                self.get_logger().info(f"  {name} activated OK")
            else:
                self.get_logger().error(f"  {name} activate FAILED")
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
