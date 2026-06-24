#!/usr/bin/env python3
"""将 nav_msgs/Odometry 转为 odom -> base_link 的 TF 广播。"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.declare_parameter('odom_topic', '/usv_1/odom')
        self.declare_parameter('odom_qos_depth', 50)

        odom_topic = self.get_parameter('odom_topic').value
        odom_qos_depth = int(self.get_parameter('odom_qos_depth').value)

        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(odom_qos_depth, 10),
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            odom_qos,
        )
        self.get_logger().info(
            f'Broadcasting TF from {odom_topic} (odom QoS depth {odom_qos.depth})'
        )

    def _odom_callback(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
