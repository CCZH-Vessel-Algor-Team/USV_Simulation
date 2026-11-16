"""Current point/drag placement, default heading and virtual-target contracts."""

import math
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

from usv_sim_full.scripts.dynamic_ship_manager_node import DynamicShipManager


def make_manager():
    node = DynamicShipManager.__new__(DynamicShipManager)
    node._ship_counter = 0
    node._read_config = Mock(return_value=(37.0, 3.0, 'box', 50.0))
    node._spawn_ship_at = Mock(return_value=True)
    node.get_logger = Mock(return_value=Mock())
    return node


def point_message():
    msg = PointStamped()
    msg.header.frame_id = 'map'
    msg.point.x, msg.point.y = 12.0, -34.0
    return msg


def pose_message(yaw):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = 12.0, -34.0
    msg.pose.pose.orientation.z = 2.0 * math.sin(yaw / 2)
    msg.pose.pose.orientation.w = 2.0 * math.cos(yaw / 2)
    return msg


@pytest.mark.parametrize('virtual', [False, True])
@pytest.mark.parametrize('heading, yaw', [
    (0.0, math.pi / 2), (90.0, 0.0), (180.0, -math.pi / 2), (270.0, -math.pi),
])
def test_click_uses_panel_heading(virtual, heading, yaw):
    node = make_manager()
    node._read_config.return_value = (heading, 3.0, 'box', 50.0)
    node.on_spawn_point(point_message(), virtual=virtual)
    node._spawn_ship_at.assert_called_once_with(
        'dyn_target_1', 12.0, -34.0, pytest.approx(yaw), 50.0, 'box', 3.0,
        virtual=virtual)


@pytest.mark.parametrize('virtual', [False, True])
@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.pi, -math.pi / 2])
def test_drag_uses_arrow_and_does_not_change_next_click(virtual, yaw):
    node = make_manager()
    node.on_spawn_pose(pose_message(yaw), virtual=virtual)
    node.on_spawn_point(point_message(), virtual=virtual)
    drag, click = node._spawn_ship_at.call_args_list
    assert drag.args == (
        'dyn_target_1', 12.0, -34.0, pytest.approx(yaw), 50.0, 'box', 3.0)
    assert click.args[0] == 'dyn_target_2'
    assert click.args[3] == pytest.approx(math.pi / 2 - math.radians(37.0))
    assert drag.kwargs == click.kwargs == {'virtual': virtual}


@pytest.mark.parametrize('virtual', [False, True])
@pytest.mark.parametrize('drag', [False, True])
@pytest.mark.parametrize('invalid', ['frame', 'nan_xy', 'infinite_z'])
def test_invalid_coordinates_do_not_create_target(virtual, drag, invalid):
    node = make_manager()
    msg = pose_message(0.0) if drag else point_message()
    pos = msg.pose.pose.position if drag else msg.point
    if invalid == 'frame':
        msg.header.frame_id = 'odom'
    elif invalid == 'nan_xy':
        pos.x = math.nan
    else:
        pos.z = math.inf
    (node.on_spawn_pose if drag else node.on_spawn_point)(msg, virtual=virtual)
    node._spawn_ship_at.assert_not_called()
    assert node._ship_counter == 0


@pytest.mark.parametrize('virtual', [False, True])
@pytest.mark.parametrize('w', [0.0, math.nan, math.inf])
def test_invalid_quaternion_does_not_create_target(virtual, w):
    node = make_manager()
    msg = pose_message(0.0)
    msg.pose.pose.orientation.w = w
    node.on_spawn_pose(msg, virtual=virtual)
    node._spawn_ship_at.assert_not_called()
    assert node._ship_counter == 0


@pytest.mark.parametrize('clear_all', [False, True])
def test_virtual_target_can_be_added_and_removed_without_gazebo(clear_all):
    node = make_manager()
    node._spawn_ship_at = DynamicShipManager._spawn_ship_at.__get__(node)
    node.backend = 'gazebo'
    node.default_mesh_profile = ''
    node._ships_lock = threading.Lock()
    node.ships, node._pending_spawns, node._missing_model_counts = {}, {}, {}
    node._retired_ships = set()
    node._membership_changed = Mock()
    node._spawn_gazebo = Mock()
    node._remove_gazebo = Mock()
    node.create_publisher = Mock()
    node.on_spawn_point(point_message(), virtual=True)
    ship = node.ships['dyn_target_1']
    assert ship.is_virtual and ship.waypoint_a == (12.0, -34.0)
    response = SimpleNamespace()
    if clear_all:
        node.on_clear(SimpleNamespace(), response)
    else:
        node.on_delete(SimpleNamespace(model_name=ship.model_name), response)
    assert response.success and not node.ships and not node._retired_ships
    assert ship._closed
    node.create_publisher.assert_not_called()
    node._spawn_gazebo.assert_not_called()
    node._remove_gazebo.assert_not_called()
