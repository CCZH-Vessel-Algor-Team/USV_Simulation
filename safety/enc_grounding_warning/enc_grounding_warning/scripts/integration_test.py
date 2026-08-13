"""Closed-loop ROS integration test without Gazebo/Nav2.

Publishes static TF, VesselState and a test plan, then verifies:
  1. /safety/ukc_state is published
  2. /safety/grounding_alerts fires when the plan crosses a shoal
  3. /safety/route_check returns first_unsafe_index >= 0
"""

from __future__ import annotations

import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from usv_interfaces.msg import VesselState

from enc_grounding_warning_msgs.msg import DepthGrid, GroundingAlert, UKCState
from enc_grounding_warning_msgs.srv import RouteCheck
from visualization_msgs.msg import MarkerArray


class IntegrationTestNode(Node):
    def __init__(self):
        super().__init__("grounding_warning_integration_test")
        self.declare_parameter("robot_frame", "usv_1/base_link")
        self.declare_parameter("boat_x", 200.0)
        self.declare_parameter("boat_y", 200.0)

        self.robot_frame = self.get_parameter("robot_frame").value
        self.boat_x = float(self.get_parameter("boat_x").value)
        self.boat_y = float(self.get_parameter("boat_y").value)

        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.vessel_pub = self.create_publisher(VesselState, "/usv_1/state/vessel", 10)
        self.plan_pub = self.create_publisher(Path, "/usv_1/plan", 10)

        self.ukc_received = False
        self.alert_received = False
        self.alert_msg = None
        self.route_grid_received = False
        self.route_markers_received = False
        self.route_grid = None
        self.route_markers = None

        self.ukc_sub = self.create_subscription(
            UKCState, "/usv_1/safety/ukc_state", self.ukc_cb, 10
        )
        self.alert_sub = self.create_subscription(
            GroundingAlert, "/usv_1/safety/grounding_alerts", self.alert_cb, 10
        )
        self.route_grid_sub = self.create_subscription(
            DepthGrid, "/usv_1/safety/route_depth_grid", self.route_grid_cb, 10
        )
        self.route_markers_sub = self.create_subscription(
            MarkerArray, "/usv_1/safety/route_depth_markers", self.route_markers_cb, 10
        )

        self.plan = self._make_plan()
        self.timer = self.create_timer(0.2, self.publish_inputs)

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.robot_frame
        t.transform.translation.x = self.boat_x
        t.transform.translation.y = self.boat_y
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _make_plan(self):
        msg = Path()
        msg.header.frame_id = "map"
        for x, y in [(self.boat_x, self.boat_y), (400.0, 300.0)]:
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def publish_inputs(self):
        now = self.get_clock().now().to_msg()

        vs = VesselState()
        vs.header.stamp = now
        vs.header.frame_id = f"{self.robot_frame}/odom"
        vs.latitude = 22.0
        vs.longitude = 114.0
        vs.pose.position.x = self.boat_x
        vs.pose.position.y = self.boat_y
        vs.pose.orientation.w = 1.0
        vs.velocity.linear.x = 2.0
        self.vessel_pub.publish(vs)

        self.plan.header.stamp = now
        self.plan_pub.publish(self.plan)

    def ukc_cb(self, msg: UKCState):
        self.ukc_received = True

    def alert_cb(self, msg: GroundingAlert):
        self.alert_received = True
        self.alert_msg = msg

    def route_grid_cb(self, msg: DepthGrid):
        self.route_grid_received = True
        self.route_grid = msg

    def route_markers_cb(self, msg: MarkerArray):
        self.route_markers_received = True
        self.route_markers = msg

    def check_route_service(self):
        client = self.create_client(RouteCheck, "/usv_1/safety/route_check")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("route_check service not available")
            return False
        req = RouteCheck.Request()
        req.route = self.plan
        future = client.call_async(req)
        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.1)
        if not future.done():
            self.get_logger().error("route_check timed out")
            return False
        resp = future.result()
        self.get_logger().info(
            f"route_check resp first_unsafe={resp.first_unsafe_index} "
            f"success={resp.success} msg={resp.message}"
        )
        if not resp.success or resp.first_unsafe_index < 0:
            self.get_logger().error(
                f"route_check failed: success={resp.success} "
                f"first_unsafe_index={resp.first_unsafe_index} msg={resp.message}"
            )
            return False
        self.get_logger().info(
            f"route_check OK: first_unsafe={resp.first_unsafe_index} "
            f"min_ukc={resp.min_ukc_m:.2f} danger=({resp.danger_x:.1f},{resp.danger_y:.1f})"
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = IntegrationTestNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    deadline = time.time() + 30.0
    try:
        while time.time() < deadline:
            time.sleep(0.2)
            if (
                node.ukc_received
                and node.alert_received
                and node.route_grid_received
                and node.route_markers_received
            ):
                break
        if not node.ukc_received:
            print("FAIL: ukc_state not received")
            return 1
        if not node.alert_received:
            print("FAIL: grounding_alert not received")
            return 1
        if not node.route_grid_received:
            print("FAIL: route_depth_grid not received")
            return 1
        if not node.route_markers_received:
            print("FAIL: route_depth_markers not received")
            return 1
        alert = node.alert_msg
        print(
            f"PASS: ukc_state received; alert level={alert.level} "
            f"distance={alert.distance_to_danger_m:.1f} m "
            f"time={alert.time_to_danger_s:.1f} s "
            f"danger=({alert.danger_x:.1f},{alert.danger_y:.1f})"
        )
        route_grid = node.route_grid
        print(
            f"PASS: route_depth_grid received "
            f"({route_grid.width}x{route_grid.height} @ {route_grid.resolution:.1f} m)"
        )
        markers = node.route_markers
        if markers.markers:
            print(
                f"PASS: route_depth_markers received "
                f"({len(markers.markers[0].points)} cubes)"
            )
        if not node.check_route_service():
            return 1
        print("PASS: closed-loop integration test")
        return 0
    finally:
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
