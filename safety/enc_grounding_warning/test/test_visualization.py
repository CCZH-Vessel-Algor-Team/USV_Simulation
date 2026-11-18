import math

import pytest
from builtin_interfaces.msg import Time
from visualization_msgs.msg import Marker

from enc_grounding_warning.markers import depth_region_markers
from enc_grounding_warning.route_depth_publisher_node import depth_to_color


def test_depth_gradient_is_warm_to_cold():
    r_shallow, g_shallow, b_shallow = depth_to_color(0.0, 0.0, 5.0)
    r_deep, g_deep, b_deep = depth_to_color(5.0, 0.0, 5.0)

    assert r_shallow > b_shallow      # 浅水偏暖
    assert b_deep > r_deep            # 深水偏冷
    assert r_shallow > r_deep


def test_depth_gradient_midpoint_is_yellowish():
    r, g, b = depth_to_color(2.5, 0.0, 5.0)
    assert r > 0.9
    assert g > 0.8


def test_user_regions_are_closed_unfilled_rings_with_the_requested_radius():
    regions = [
        {"x": 10.0, "y": -4.0, "radius": 3.0, "depth_m": 0.8},
        {"x": -20.0, "y": 5.0, "radius": 12.0, "depth_m": 2.0},
    ]
    message = depth_region_markers({"enabled": True, "shoals": regions}, "map", Time())
    assert len(message.markers) == 1
    marker = message.markers[0]
    assert marker.action == Marker.ADD
    assert marker.type == Marker.LINE_LIST  # no filled disks, labels or 3D pins
    assert marker.header.frame_id == "map"
    assert 0.2 < marker.scale.x <= 0.4
    assert 0.4 < marker.color.r < 0.7
    assert 0.0 <= marker.color.g < 0.2
    assert 0.0 <= marker.color.b < 0.2
    assert 0.0 < marker.color.a < 1.0
    assert 0.0 < marker.pose.position.z < 0.1
    points_per_region = len(marker.points) // len(regions)
    for index, region in enumerate(regions):
        points = marker.points[index * points_per_region:(index + 1) * points_per_region]
        assert points[0] == points[-1]
        for point in points:
            assert math.hypot(point.x - region["x"], point.y - region["y"]) == pytest.approx(region["radius"])
            assert point.z == 0.0


@pytest.mark.parametrize("settings", [
    {"enabled": False, "shoals": [{"x": 0.0, "y": 0.0, "radius": 30.0, "depth_m": 0.8}]},
    {"enabled": True, "shoals": []},
])
def test_no_user_regions_explicitly_clear_the_old_ring_batch(settings):
    message = depth_region_markers(settings, "map", Time())
    assert len(message.markers) == 1
    assert message.markers[0].action == Marker.DELETE
    assert not message.markers[0].points


def test_deleting_a_region_replaces_the_same_batch_without_leaving_old_rings():
    regions = [
        {"x": 10.0, "y": 0.0, "radius": 3.0, "depth_m": 0.8},
        {"x": -20.0, "y": 0.0, "radius": 8.0, "depth_m": 1.0},
    ]
    before = depth_region_markers({"enabled": True, "shoals": regions}, "map", Time()).markers[0]
    after = depth_region_markers({"enabled": True, "shoals": regions[1:]}, "map", Time()).markers[0]
    cleared = depth_region_markers({"enabled": False, "shoals": []}, "map", Time()).markers[0]
    assert (before.ns, before.id) == (after.ns, after.id) == (cleared.ns, cleared.id)
    assert len(after.points) * 2 == len(before.points)
    assert all(math.hypot(point.x + 20.0, point.y) == pytest.approx(8.0) for point in after.points)
