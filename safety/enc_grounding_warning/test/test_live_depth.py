"""Exercise plugin service requests against the real safety nodes."""

import time

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster
from usv_interfaces.msg import VesselState
from visualization_msgs.msg import Marker, MarkerArray

from enc_grounding_warning_msgs.msg import DepthGrid, GroundingAlert, UKCState
from enc_grounding_warning_msgs.srv import ConfigureDepth
from enc_grounding_warning.depth_provider_node import DepthProviderNode
from enc_grounding_warning.grounding_warning_node import GroundingWarningNode
from enc_grounding_warning.live_depth import LiveDepthProvider, settings_to_msg
from enc_grounding_warning.route_depth_publisher_node import RouteDepthPublisherNode
from enc_grounding_warning.ukc_estimator_node import UkcEstimatorNode


def test_edit_sync_and_session_only_restart(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    grid = tmp_path / "grid.yaml"
    grid.write_text(yaml.safe_dump({"depth_grid": {
        "origin_x": -40.0, "origin_y": -40.0, "resolution": 1.0,
        "width": 80, "height": 80, "depth_m": 5.0,
        "shoals": [{"x": 25.0, "y": 25.0, "radius": 4.0, "depth_m": 1.1}],
    }}))
    params = tmp_path / "params.yaml"
    params.write_text(yaml.safe_dump({
        "ukc": {"static_draft_m": 0.5, "heel_allowance_m": 0.05,
                "trim_allowance_m": 0.03, "route_corridor_half_width_m": 3.0},
        "grid_publish": {"grounding_risk_grid": False},
    }))
    rclpy.init(args=[
        "--ros-args", "-r", "__ns:=/depth_editor_test",
        "-p", f"depth_grid_file:={grid}", "-p", f"params_file:={params}",
        "-p", "near_extent_m:=20.0", "-p", "near_resolution_m:=1.0",
    ])
    executor = SingleThreadedExecutor()
    nodes = []

    def add(node):
        nodes.append(node)
        executor.add_node(node)
        return node

    def until(condition, seconds=12):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if condition():
                return
        raise AssertionError("Timed out waiting for live depth update")

    try:
        authority = add(DepthProviderNode())
        ukc = add(UkcEstimatorNode())
        warning = add(GroundingWarningNode())
        route = add(RouteDepthPublisherNode())
        observer = add(Node("depth_test_observer"))
        messages = {}
        retained = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for topic, message_type, key in [
            ("safety/ukc_state", UKCState, "ukc"),
            ("safety/depth_grid", DepthGrid, "near"),
            ("safety/route_depth_grid", DepthGrid, "route"),
            ("safety/grounding_alerts", GroundingAlert, "alert"),
            ("safety/depth_region_markers", MarkerArray, "regions"),
        ]:
            observer.create_subscription(
                message_type, topic, lambda msg, key=key: messages.update({key: msg}),
                retained if key == "regions" else 10,
            )
        broadcaster = StaticTransformBroadcaster(observer)
        transform = TransformStamped()
        transform.header.frame_id = "map"
        transform.child_frame_id = "usv_1/base_link"
        transform.transform.rotation.w = 1.0
        broadcaster.sendTransform(transform)
        state_pub = observer.create_publisher(VesselState, "state/vessel", 10)
        plan_pub = observer.create_publisher(Path, "plan", 10)
        state = VesselState()
        state.pose.orientation.w = 1.0
        state.velocity.linear.x = 1.0
        plan = Path()
        plan.header.frame_id = "map"
        for x in (0.0, 20.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            plan.poses.append(pose)

        def publish_inputs():
            state_pub.publish(state)
            plan_pub.publish(plan)

        observer.create_timer(0.1, publish_inputs)
        client = observer.create_client(ConfigureDepth, "safety/configure_depth")
        until(client.service_is_ready)

        def configure(settings=None):
            request = ConfigureDepth.Request()
            request.apply = settings is not None
            if settings is not None:
                request.settings = settings_to_msg(settings)
            future = client.call_async(request)
            until(future.done)
            return future.result()

        assert not configure().settings.enabled
        assert not configure().settings.shoals
        assert authority.provider.query_point(25, 25)["depth"] == pytest.approx(1.1)
        until(lambda: "regions" in messages)
        assert messages["regions"].markers[0].action == Marker.DELETE  # presets are not drawn
        custom = {"enabled": True, "base_depth_m": 5.0, "shoals": [
            {"x": 0.0, "y": 0.0, "radius": 6.0, "depth_m": 0.6},
        ]}
        assert configure(custom).success
        until(lambda: (
            messages.get("ukc") is not None and messages["ukc"].risk_level >= 2
            and abs(messages["ukc"].chart_depth_m - 0.6) < 1e-5
            and messages.get("alert") is not None and messages["alert"].level > 0
            and messages.get("near") is not None and min(messages["near"].depth_m) < 0.7
            and messages.get("route") is not None
            and min(d for d in messages["route"].depth_m if d >= 0) < 0.7
            and messages["regions"].markers[0].action == Marker.ADD
        ))
        assert len(messages["regions"].markers[0].points) == 192  # one user circle only
        late_rings = []
        observer.create_subscription(
            MarkerArray, "safety/depth_region_markers", late_rings.append, retained
        )
        until(lambda: bool(late_rings))
        assert late_rings[-1].markers[0].action == Marker.ADD
        assert len(late_rings[-1].markers[0].points) == 192
        for node in (authority, ukc, warning, route):
            assert node.provider.settings == custom

        # A consumer joining later must receive the retained live settings.
        late = add(Node("late_depth_consumer"))
        late_provider = LiveDepthProvider(late, str(grid))
        until(lambda: late_provider.query_point(0, 0)["depth"] == pytest.approx(0.6))
        # Invalid editor input must leave the active settings intact.
        invalid = {**custom, "base_depth_m": -1.0}
        assert not configure(invalid).success
        assert authority.provider.settings == custom

        safe = {"enabled": True, "base_depth_m": 5.0, "shoals": []}
        assert configure(safe).success
        until(lambda: (
            messages["ukc"].chart_depth_m == 5.0 and messages["ukc"].risk_level == 0
            and messages["alert"].level == 0
            and messages["regions"].markers[0].action == Marker.DELETE
        ))

        assert configure(custom).success
        until(lambda: messages["regions"].markers[0].action == Marker.ADD)

        # Recreate the authority: user regions are session-only, so restarting
        # the node returns to the original grid settings.
        executor.remove_node(authority)
        nodes.remove(authority)
        authority.destroy_node()
        old_ring_stamp = messages["regions"].markers[0].header.stamp
        authority = add(DepthProviderNode())
        assert not authority.provider.settings["enabled"]
        assert authority.provider.query_point(0, 0)["depth"] == pytest.approx(5.0)
        assert authority.provider.query_point(25, 25)["depth"] == pytest.approx(1.1)
        assert configure().settings.base_depth_m == 5.0
        until(lambda: messages["regions"].markers[0].header.stamp != old_ring_stamp)
        assert messages["regions"].markers[0].action == Marker.DELETE

        restored = configure({**safe, "enabled": False})
        assert restored.success
        assert not restored.settings.enabled
        assert restored.settings.base_depth_m == 5.0
        assert not restored.settings.shoals
        until(lambda: ukc.provider.query_point(0, 0)["depth"] == 5.0
              and messages["regions"].markers[0].action == Marker.DELETE)
        executor.remove_node(authority)
        nodes.remove(authority)
        authority.destroy_node()
        authority = add(DepthProviderNode())
        assert not authority.provider.settings["enabled"]
        assert authority.provider.query_point(0, 0)["depth"] == 5.0
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
