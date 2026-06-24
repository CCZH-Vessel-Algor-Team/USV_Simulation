#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_msgs.msg import TFMessage


class TfNamespaceRelay(Node):
    def __init__(self):
        super().__init__('tf_namespace_relay')

        self.declare_parameter('namespace', 'usv_1')
        ns = self.get_parameter('namespace').get_parameter_value().string_value.strip('/')
        self.namespace = ns if ns else 'usv_1'

        tf_topic_out = f'/{self.namespace}/tf'
        tf_static_topic_out = f'/{self.namespace}/tf_static'

        # Upstream /tf is commonly best-effort in sim pipelines.
        tf_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Nav2 TF listeners may request reliable QoS on namespaced /tf.
        tf_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Match typical /tf_static QoS: transient local + reliable
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_pub = self.create_publisher(TFMessage, tf_topic_out, tf_pub_qos)
        self.tf_static_pub = self.create_publisher(TFMessage, tf_static_topic_out, tf_static_qos)

        # Keep a merged static TF snapshot so late-join subscribers receive a complete tree.
        self._static_tf_cache = {}

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self._tf_cb, tf_sub_qos)
        self.tf_static_sub = self.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, tf_static_qos)

        self.get_logger().info(
            f"Relaying TF topics: /tf -> {tf_topic_out}, /tf_static -> {tf_static_topic_out}"
        )

    def _tf_cb(self, msg: TFMessage):
        self.tf_pub.publish(msg)

    def _tf_static_cb(self, msg: TFMessage):
        for transform in msg.transforms:
            key = (transform.header.frame_id, transform.child_frame_id)
            self._static_tf_cache[key] = transform

        merged_msg = TFMessage()
        merged_msg.transforms = list(self._static_tf_cache.values())
        self.tf_static_pub.publish(merged_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfNamespaceRelay()
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
