import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from rclpy.time import Time

from usv_sim_full.scripts.storm_field_manager_node import (
    StormField,
    StormFieldManager,
)


class _ManagerFixture:
    def __init__(self, storm):
        self.storms = {storm.name: storm}
        self.parameters = {}

    def set_parameters(self, parameters):
        self.parameters = {parameter.name: parameter.value for parameter in parameters}


def test_config_changes_apply_only_to_future_storms():
    storm = StormField(
        'storm_1', 10.0, 20.0, 30.0, 0.0, 1.0, 600.0, 100.0, 0.0)
    manager = _ManagerFixture(storm)
    request = SimpleNamespace(
        radius=80.0,
        drift_heading_deg=90.0,
        drift_speed=2.0,
        weather_validity_duration_s=1200.0,
        weather_grid_resolution_m=200.0,
    )
    response = SimpleNamespace()

    StormFieldManager.on_set_config(manager, request, response)

    assert response.success
    assert storm.radius == 30.0
    assert storm.drift_heading_deg == 0.0
    assert storm.drift_speed == 1.0
    assert manager.parameters == {
        'drift_heading_deg': 90.0,
        'drift_speed': 2.0,
        'radius': 80.0,
        'weather_validity_duration_s': 1200.0,
        'weather_grid_resolution_m': 200.0,
    }


def make_manager():
    manager = StormFieldManager.__new__(StormFieldManager)
    manager.frame_id = 'map'
    manager.storms = {}
    manager._read_config = Mock(return_value=(37.0, 2.0, 80.0, 1200.0, 200.0))
    manager.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=10_000_000_000))
    manager.get_logger = Mock(return_value=Mock())
    return manager


@pytest.mark.parametrize('yaw, heading, movement', [
    (0.0, 90.0, (2.0, 0.0)), (math.pi / 2, 0.0, (0.0, 2.0)),
    (math.pi, 270.0, (-2.0, 0.0)), (-math.pi / 2, 180.0, (0.0, -2.0)),
])
def test_drag_sets_drift_direction_without_overwriting_click_defaults(yaw, heading, movement):
    manager = make_manager()
    point = PointStamped()
    point.header.frame_id = 'map'
    manager.on_clicked_point(point)
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x, pose.pose.pose.position.y = 12.0, -34.0
    pose.pose.pose.orientation.z = 2.0 * math.sin(yaw / 2)
    pose.pose.pose.orientation.w = 2.0 * math.cos(yaw / 2)
    manager.on_spawn_pose(pose)
    manager.on_clicked_point(point)
    first, dragged, last = manager.storms.values()
    assert first.drift_heading_deg == last.drift_heading_deg == 37.0
    assert dragged.drift_heading_deg == pytest.approx(heading)
    assert (dragged.radius, dragged.drift_speed,
            dragged.weather_validity_duration_s, dragged.weather_grid_resolution_m,
            dragged.created_time_sec) == (80.0, 2.0, 1200.0, 200.0, 10.0)
    dragged.step(1.0)
    assert (dragged.x, dragged.y) == pytest.approx((12.0 + movement[0], -34.0 + movement[1]))


@pytest.mark.parametrize('drag', [False, True])
@pytest.mark.parametrize('invalid', ['frame', 'nan_position', 'infinite_z'])
def test_invalid_storm_placement_is_rejected(drag, invalid):
    manager = make_manager()
    msg = PoseWithCovarianceStamped() if drag else PointStamped()
    msg.header.frame_id = 'odom' if invalid == 'frame' else 'map'
    if drag:
        msg.pose.pose.orientation.w = 1.0
    position = msg.pose.pose.position if drag else msg.point
    if invalid == 'nan_position':
        position.x = math.nan
    if invalid == 'infinite_z':
        position.z = math.inf
    (manager.on_spawn_pose if drag else manager.on_clicked_point)(msg)
    assert not manager.storms


@pytest.mark.parametrize('w', [0.0, math.nan, math.inf])
def test_invalid_storm_quaternion_is_rejected(w):
    manager = make_manager()
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.orientation.w = w
    manager.on_spawn_pose(msg)
    assert not manager.storms


def test_control_publishes_consistent_weather_tracks_names_and_markers():
    manager = make_manager()
    manager.dt = 0.1
    manager.tracked_pub = Mock()
    manager.storm_field_pub = Mock()
    manager.names_pub = Mock()
    manager.marker_pub = Mock()
    point = PointStamped()
    point.header.frame_id = 'map'
    manager.on_clicked_point(point)
    manager.control_loop()
    tracks = manager.tracked_pub.publish.call_args.args[0]
    weather = manager.storm_field_pub.publish.call_args.args[0]
    markers = manager.marker_pub.publish.call_args.args[0]
    assert len(tracks.obstacles) == len(weather.storms) == 1
    assert weather.header == tracks.header
    assert tracks.header.stamp.sec == 10
    assert weather.storms[0].remaining_validity_s == 1200.0
    assert weather.storms[0].center == tracks.obstacles[0].pose.position
    assert manager.names_pub.publish.call_args.args[0].data == '["storm_1"]'
    assert markers.markers[0].action == markers.markers[0].DELETEALL
    assert len(markers.markers) == 3


def test_delete_clear_and_expiration_preserve_other_storms():
    manager = make_manager()
    for _ in range(3):
        manager._spawn_storm(0.0, 0.0)
    response = SimpleNamespace()
    manager.on_delete(SimpleNamespace(storm_name='missing'), response)
    assert not response.success and len(manager.storms) == 3
    manager.on_delete(SimpleNamespace(storm_name='storm_2'), response)
    assert response.success and list(manager.storms) == ['storm_1', 'storm_3']
    manager.storms['storm_3'].weather_validity_duration_s = 0.0
    manager.remove_expired_storms(1210.0)
    assert list(manager.storms) == ['storm_3']
    manager.on_clear(SimpleNamespace(), response)
    assert response.success and not manager.storms
