import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import importlib
import subprocess
import threading
import time

class MonitorNode(Node):
    def __init__(self):
        super().__init__('sim_test_monitor')
        self.active_nodes = []
        self.topic_hz = {}
        self.topic_counts = {}
        self.locked_topics = set()
        self.topic_types = {}
        self.active_gz_topics = set()
        self.subs = {}

        self.hz_timer = self.create_timer(1.0, self.calc_hz_callback)
        self.discovery_timer = self.create_timer(1.0, self.discovery_callback)
        self.gz_discovery_timer = self.create_timer(2.0, self.gz_discovery_callback)

    def import_msg_class(self, type_str):
        try:
            parts = type_str.split('/')
            if len(parts) == 3:
                mod = importlib.import_module(f"{parts[0]}.{parts[1]}")
                return getattr(mod, parts[2])
            elif len(parts) == 2:
                mod = importlib.import_module(f"{parts[0]}.msg")
                return getattr(mod, parts[1])
        except Exception as e:
            self.get_logger().error(f"Error importing {type_str}: {e}")
        return None

    def gz_discovery_callback(self):
        def task():
            try:
                res = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=1.5)
                if res.returncode == 0:
                    topics = [t.strip() for t in res.stdout.split(chr(10)) if t.strip()]
                    self.active_gz_topics = set(topics)
            except Exception as e:
                pass
        threading.Thread(target=task, daemon=True).start()

    def discovery_callback(self):
        self.active_nodes = self.get_node_names()
        
        for topic_name, types in self.get_topic_names_and_types():
            if not types:
                continue
            t_type_str = types[0]
            self.topic_types[topic_name] = t_type_str
            
            if topic_name not in self.locked_topics:
                msg_class = self.import_msg_class(t_type_str)
                if msg_class:
                    qos_profile = QoSProfile(
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=5,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT
                    )
                    cb = self.make_callback(topic_name)
                    try:
                        self.get_logger().info(f"Subscribing to: {topic_name} with type {t_type_str}")
                        sub = self.create_subscription(msg_class, topic_name, cb, qos_profile)
                        self.subs[topic_name] = sub
                        self.locked_topics.add(topic_name)
                        self.topic_counts[topic_name] = 0
                        self.topic_hz[topic_name] = 0.0
                    except Exception as e:
                        self.get_logger().error(f"Cannot sub to {topic_name}: {e}")
                else:
                    self.get_logger().error(f"Could not find msg class for {t_type_str} on {topic_name}")

    def make_callback(self, topic_name):
        def cb(msg):
            # Try to catch if this is ever called!
            if topic_name in self.topic_counts:
                self.topic_counts[topic_name] += 1
        return cb

    def calc_hz_callback(self):
        for topic, count in self.topic_counts.items():
            self.topic_hz[topic] = float(count)
            self.topic_counts[topic] = 0

    def get_status(self):
        return {
            'nodes': self.active_nodes,
            'topics_hz': self.topic_hz,
            'topics_type': self.topic_types,
            'gz_topics': self.active_gz_topics
        }

def spin_thread(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
