"""TF broadcast: map→nav_map (static identity) + nav_map→base_link (from /odin1/relocation)."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OdomToTfNode(Node):
    def __init__(self):
        super().__init__("odom_to_tf_node")
        self.declare_parameter("pose_topic", "/odin1/relocation")
        pose_topic = self.get_parameter("pose_topic").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        now = self.get_clock().now().to_msg()

        # map → nav_map: world origin → Nav2 global frame
        static_map = TransformStamped()
        static_map.header.stamp = now
        static_map.header.frame_id = "map"
        static_map.child_frame_id = "nav_map"
        static_map.transform.rotation.w = 1.0

        # nav_map → odom: Nav2 defaults to "odom" as local frame;
        # make it an identity child so base_link→odom lookups succeed
        static_odom = TransformStamped()
        static_odom.header.stamp = now
        static_odom.header.frame_id = "nav_map"
        static_odom.child_frame_id = "odom"
        static_odom.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([static_map, static_odom])

        self.sub = self.create_subscription(PoseStamped, pose_topic, self.pose_cb, 10)
        self.get_logger().info(f"TF: map→nav_map + nav_map→odom (static) + nav_map→base_link (from {pose_topic})")

    def pose_cb(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "nav_map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
