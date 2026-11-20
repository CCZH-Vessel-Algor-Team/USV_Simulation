"""Service, measured-frame and placement tests without a ROS graph or Gazebo."""
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
from unittest.mock import Mock
import uuid

import pytest
from geometry_msgs.msg import Pose
from usv_interfaces.msg import Buoy, BuoyArray
from usv_interfaces.srv import (
    ListDynamicShipModels, SetDynamicShipConfig, SpawnDynamicShip,
)

from usv_sim_full.dynamic_ship_catalog import ShipCatalog
from usv_sim_full.scripts.dynamic_ship_manager_node import DynamicShip, DynamicShipManager


@pytest.fixture
def manager():
    m = DynamicShipManager.__new__(DynamicShipManager)
    m.__dict__.update(
        catalog=ShipCatalog(Path(__file__).parents[1]/'description/models/target_fleet/catalog.json'),
        _catalog_error='', _selected_model_id='legacy_10m', _ship_counter=0,
        ships={}, _pending_spawns={}, _ships_lock=threading.Lock(),
        _clock_reset=threading.Event(), _registry_revision=0, _last_published_stamp_ns=None,
        _buoy_snapshot=None, count_publishers=Mock(return_value=0),
        backend='gazebo', marker_pub=None, dt=.1, pose_feedback_timeout=1.,
        tracked_pub=Mock(), snapshot_pub=Mock(), names_pub=Mock())
    m.get_logger = Mock(return_value=Mock())
    m.set_parameters_atomically = Mock(return_value=NS(successful=True))
    m._spawn_ship_at = Mock(return_value=True)
    m.get_parameter = Mock(side_effect=lambda name: NS(value={
        'fleet_spawn_check_map': False, 'ownship_model': '',
    }[name]))
    m.now_ns = 1_000_000_000
    m.get_clock = lambda: NS(now=lambda: NS(nanoseconds=m.now_ns))
    cache = NS(_lock=threading.Lock(), _frame=None, _awaiting_absence=set(), samples={})
    cache.snapshot = lambda: (dict(cache.samples), cache._frame)
    cache._promote_pending = lambda now: None
    m.gz_pose_cache = cache
    return m


def make_ship(manager, name, model_id, x=0., y=0., virtual=False):
    pose = Pose()
    pose.position.x, pose.position.y, pose.orientation.w = x, y, 1.
    spec = manager.catalog.get(model_id)
    return DynamicShip(name, str(uuid.uuid5(uuid.NAMESPACE_DNS, name)), pose, 100.,
                       spec.backend, 1., (1., 0., 0.), '', Mock(), '', 'test', spec,
                       is_virtual=virtual)


def test_directory_and_config_do_not_modify_existing_instances(manager):
    m = manager
    existing = make_ship(m, 'first', 'legacy_10m')
    m.ships['first'] = existing
    response = DynamicShipManager.on_list_models(m, ListDynamicShipModels.Request(), ListDynamicShipModels.Response())
    assert response.success and len(response.models) == 13
    assert response.default_model_id == 'legacy_10m'
    req = SetDynamicShipConfig.Request(model_id='munich_maersk', speed=1., half_distance=500., heading_deg=90.)
    response = m.on_set_config(req, SetDynamicShipConfig.Response())
    assert response.success and response.model_id == 'munich_maersk'
    assert m._selected_model_id == 'munich_maersk'
    assert existing.model_spec.model_id == 'legacy_10m'
    assert existing.speed == 1. and existing.half_distance == 100.


@pytest.mark.parametrize('field,value', [('model_id', 'missing'), ('speed', float('nan')),
                                        ('speed', -1.), ('half_distance', 0.)])
def test_failed_config_keeps_previous_selection(manager, field, value):
    req = SetDynamicShipConfig.Request(model_id='vb_rumba', speed=1., half_distance=100.)
    setattr(req, field, value)
    response = manager.on_set_config(req, SetDynamicShipConfig.Response())
    assert not response.success and response.message
    assert manager._selected_model_id == 'legacy_10m'
    manager.set_parameters_atomically.assert_not_called()


def test_rejected_parameters_cannot_acknowledge_new_model(manager):
    manager.set_parameters_atomically.return_value = NS(successful=False, reason='read only')
    response = manager.on_set_config(
        SetDynamicShipConfig.Request(model_id='vb_rumba', speed=1., half_distance=100.),
        SetDynamicShipConfig.Response())
    assert not response.success and manager._selected_model_id == 'legacy_10m'


def test_config_and_spawn_still_select_legacy_shapes_and_mesh_profile(manager):
    response = manager.on_set_config(
        SetDynamicShipConfig.Request(model_id='debug_box', speed=3., half_distance=100.),
        SetDynamicShipConfig.Response())
    assert response.success and manager._selected_model_id == 'debug_box'
    req = SpawnDynamicShip.Request(name='with_profile', model_id='legacy_10m',
                                  speed=3., half_distance=100., mesh_profile='/test/profile.yaml')
    req.pose.orientation.w = 1.
    response = manager.on_spawn(req, SpawnDynamicShip.Response())
    assert response.success
    assert manager._spawn_ship_at.call_args.args[-2] == 'legacy_10m'
    assert manager._spawn_ship_at.call_args.kwargs == {
        'virtual': False, 'mesh_profile': '/test/profile.yaml'}


@pytest.mark.parametrize('virtual', [False, True])
def test_spawn_has_explicit_model_without_changing_future_click_config(manager, virtual):
    req = SpawnDynamicShip.Request(
        name='big_one', model_id='munich_maersk', speed=1., half_distance=500.,
        virtual_ship=virtual)
    req.pose.orientation.w = 1.
    response = manager.on_spawn(req, SpawnDynamicShip.Response())
    assert response.success and response.model_name == 'big_one'
    assert manager._spawn_ship_at.call_args.args[-2] == 'munich_maersk'
    assert manager._spawn_ship_at.call_args.kwargs == {'virtual': virtual, 'mesh_profile': None}
    assert manager._selected_model_id == 'legacy_10m'


def set_frame(manager, stamp_ns, poses):
    manager.now_ns = stamp_ns
    manager.gz_pose_cache.samples = {name: (*p, stamp_ns) for name, p in poses.items()}
    manager.gz_pose_cache._frame = (stamp_ns, dict(manager.gz_pose_cache.samples))


def test_snapshot_geometry_id_velocity_and_nav_radius_are_one_measured_frame(manager):
    m = manager
    m.ships = {name: make_ship(m, name, model) for name, model in
               [('small', 'vb_rumba'), ('big', 'munich_maersk'), ('same_type', 'vb_rumba')]}
    set_frame(m, 1_000_000_000, {'small': (0., 0., 0.), 'big': (1000., 0., .3), 'same_type': (2000., 0., 0.)})
    DynamicShipManager.control_loop(m)
    m.snapshot_pub.publish.assert_not_called()  # velocity bootstrap is not a complete frame
    set_frame(m, 1_100_000_000, {'small': (.1, 0., 0.), 'big': (1000.2, 0., .3), 'same_type': (2000.1, 0., 0.)})
    DynamicShipManager.control_loop(m)
    snapshot = m.snapshot_pub.publish.call_args.args[0]
    tracked = m.tracked_pub.publish.call_args.args[0]
    assert snapshot.header == tracked.header
    assert snapshot.header.stamp.nanosec == 100_000_000
    assert len(snapshot.ships) == 3
    assert len({bytes(s.target_id.uuid) for s in snapshot.ships}) == 3
    for observation, target in zip(snapshot.ships, tracked.ships):
        spec = m.ships[observation.model_name].model_spec
        assert (observation.size_l, observation.size_w, observation.size_h) == (spec.length, spec.beam, spec.height)
        assert observation.target_id == target.target_id
        assert observation.radius == target.radius == spec.radius
        assert observation.pose == target.pose and observation.twist == target.twist
        assert observation.twist.linear.x > 0
    m.snapshot_pub.publish.reset_mock()
    set_frame(m, 1_200_000_000, {'small': (.2, 0., 0.), 'big': (1000.3, 0., .3)})
    DynamicShipManager.control_loop(m)
    m.snapshot_pub.publish.assert_not_called()  # no partial frames / geometry races


def test_empty_scene_produces_empty_complete_snapshot(manager):
    set_frame(manager, 1_000_000_000, {})
    DynamicShipManager.control_loop(manager)
    assert manager.snapshot_pub.publish.call_args.args[0].ships == []


def test_replaced_gazebo_frame_does_not_publish_empty_snapshot(manager):
    set_frame(manager, 1_000_000_000, {})
    cache = manager.gz_pose_cache
    cache._promote_pending = lambda now: setattr(cache, '_frame', (now, {}))
    manager.control_loop()
    manager.tracked_pub.publish.assert_not_called()
    manager.snapshot_pub.publish.assert_not_called()


def test_virtual_catalog_ship_moves_without_gazebo_and_reports_dimensions(manager):
    m = manager
    m.backend, m.gz_pose_cache = 'virtual', None
    ship = make_ship(m, 'virtual_large', 'munich_maersk', virtual=True)
    m.ships[ship.model_name] = ship
    m.control_loop()
    snapshot = m.snapshot_pub.publish.call_args.args[0]
    tracked = m.tracked_pub.publish.call_args.args[0]
    assert snapshot.header == tracked.header
    assert snapshot.header.stamp.sec == 1
    observation, = snapshot.ships
    assert observation.model_id == 'munich_maersk'
    assert observation.size_l == ship.model_spec.length
    assert observation.pose.position.x > 0 and observation.twist.linear.x > 0
    assert observation.radius == tracked.ships[0].radius == ship.model_spec.radius
    assert ship.cmd_vel_pub is None


def test_mixed_snapshot_waits_for_physics_and_keeps_virtual_catalog_ship(manager):
    m = manager
    real = make_ship(m, 'real', 'vb_rumba')
    virtual = make_ship(m, 'virtual', 'munich_maersk', virtual=True)
    m.ships = {ship.model_name: ship for ship in (real, virtual)}
    set_frame(m, 1_000_000_000, {'real': (0., 0., 0.)})
    m.control_loop()
    m.snapshot_pub.publish.assert_not_called()
    set_frame(m, 1_100_000_000, {'real': (.1, 0., 0.)})
    m.control_loop()
    snapshot = m.snapshot_pub.publish.call_args.args[0]
    assert snapshot.header.stamp.nanosec == 100_000_000
    assert [(s.model_name, s.model_id) for s in snapshot.ships] == [
        ('real', 'vb_rumba'), ('virtual', 'munich_maersk')]


def test_fleet_placement_fails_closed_for_missing_map_ownship_and_other_feedback(manager):
    m, model = manager, manager.catalog.get('munich_maersk')
    m.get_parameter.side_effect = lambda name: NS(value=name == 'fleet_spawn_check_map')
    m._spawn_map = None
    with pytest.raises(ValueError, match='Map unavailable'):
        m._validate_fleet_placement(model, 0., 0., 0.)
    m.get_parameter.side_effect = lambda name: NS(value='usv_1' if name == 'ownship_model' else False)
    with pytest.raises(ValueError, match='Ownship feedback'):
        m._validate_fleet_placement(model, 0., 0., 0.)
    set_frame(m, 1_000_000_000, {'usv_1': (190., 0., 0.)})
    with pytest.raises(ValueError, match='overlaps ownship'):
        m._validate_fleet_placement(model, 0., 0., 0.)
    m.ships['virtual'] = make_ship(m, 'virtual', 'munich_maersk', 1000., virtual=True)
    m._validate_fleet_placement(model, 1000., 0., 0.)
    m.ships['other'] = make_ship(m, 'other', 'vb_rumba', 1000.)
    with pytest.raises(ValueError, match='fresh placement feedback'):
        m._validate_fleet_placement(model, 1000., 0., 0.)
    set_frame(m, 1_000_000_000, {'usv_1': (190., 0., 0.), 'other': (1190., 0., 0.)})
    with pytest.raises(ValueError, match='overlaps dynamic target'):
        m._validate_fleet_placement(model, 1000., 0., 0.)


def test_large_ship_turn_waits_for_measured_heading_not_a_fixed_timer(manager):
    ship = make_ship(manager, 'big', 'munich_maersk')
    assert ship.compute_cmd_vel(.1).linear.x == 0.  # no feedback
    ship.feedback_fresh = True
    ship.current_x = 197.
    cmd = ship.compute_cmd_vel(.1)
    assert ship.direction == -1 and ship._fleet_turning
    assert cmd.linear.x == 0. and abs(cmd.angular.y) == pytest.approx(math.pi)
    for _ in range(1000):
        assert ship.compute_cmd_vel(.1).linear.x == 0.
    ship.actual_yaw = math.pi
    assert ship.compute_cmd_vel(.1).linear.x > 0.
    assert not ship._fleet_turning


def test_fleet_placement_checks_optional_buoy_manager_and_stale_snapshots(manager):
    m, model = manager, manager.catalog.get('munich_maersk')
    m.count_publishers.return_value = 1
    with pytest.raises(ValueError, match='Await the active buoy'):
        m._validate_fleet_placement(model, 0., 0., 0.)
    msg = BuoyArray()
    msg.header.frame_id, msg.header.stamp.sec = 'map', 1
    buoy = Buoy(name='buoy_at_bow', radius_m=2.)
    buoy.pose.position.x = 199.
    msg.buoys = [buoy]
    m._buoy_snapshot = msg
    with pytest.raises(ValueError, match='overlaps dynamic buoy'):
        m._validate_fleet_placement(model, 0., 0., 0.)
    m._validate_fleet_placement(model, 1000., 0., 0.)
    m.now_ns = 3_000_000_000
    with pytest.raises(ValueError, match='snapshot is stale'):
        m._validate_fleet_placement(model, 1000., 0., 0.)
