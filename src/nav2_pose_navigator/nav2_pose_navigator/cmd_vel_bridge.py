"""Twist (/cmd_vel) → Float32MultiArray (/t0x0101_action) [vx, vy, wz]."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, "/t0x0101_action", 10)
        self.get_logger().info("cmd_vel_bridge: /cmd_vel -> /t0x0101_action [vx, vy, wz]")

    def cb(self, msg: Twist):
        out = Float32MultiArray()
        out.data = [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)]
        self.pub.publish(out)


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
