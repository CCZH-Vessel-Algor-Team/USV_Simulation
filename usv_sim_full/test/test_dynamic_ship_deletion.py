"""Keep deletion and snapshot fences intact when scene query transport changes."""

import json
import threading
from types import SimpleNamespace
from unittest.mock import Mock

from geometry_msgs.msg import Pose, Twist

from usv_sim_full.scripts.dynamic_ship_manager_node import (
    DynamicShip, DynamicShipManager, GazeboPoseCache,
)


def make_manager():
    now = SimpleNamespace(nanoseconds=10_000_000_000)
    cache = GazeboPoseCache.__new__(GazeboPoseCache)
    cache._lock = threading.Lock()
    cache._samples = {}
    cache._frame = cache._pending_frame = None
    cache._watermark_ns = cache._max_observed_ns = None
    cache._not_before_ns = 0
    cache._awaiting_absence = set()
    cache._now_ns = lambda: now.nanoseconds

    ship = DynamicShip.__new__(DynamicShip)
    ship.model_name = 'dyn_target_1'
    ship.is_virtual = False
    ship.model_spec = None
    ship.target_id = '12345678-9abc-def0-1234-56789abcdef0'
    ship._publisher_lock = threading.Lock()
    ship._closed = False
    ship.cmd_vel_pub = Mock()
    ship.cleanup = Mock()
    ship.sync_from_feedback_ns = Mock(return_value=True)
    ship.compute_cmd_vel = Mock(return_value=Twist())
    ship.get_current_pose = Mock(return_value=Pose())
    ship.get_current_twist = Mock(return_value=Twist())
    ship.pose_tracker = SimpleNamespace(ready=True, stamp_ns=now.nanoseconds, velocity=(0., 0.))

    node = DynamicShipManager.__new__(DynamicShipManager)
    node.backend = 'gazebo'
    node._ships_lock = threading.Lock()
    node.ships = {ship.model_name: ship}
    node._pending_spawns = {}
    node._retired_ships = set()
    node._missing_model_counts = {}
    node._registry_revision = 0
    node.gz_pose_cache = cache
    node._clock_reset = threading.Event()
    node._last_published_stamp_ns = None
    node.pose_feedback_timeout = 1.0
    node.dt = 0.1
    node.tracked_pub = Mock()
    node.snapshot_pub = Mock()
    node.names_pub = Mock()
    node.marker_pub = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: now)
    node.get_logger = Mock(return_value=Mock())
    node._gazebo_model_names = Mock()
    return node, ship, cache, now


def test_external_delete_requires_two_confirmations_and_stops_ship_information():
    node, ship, cache, now = make_manager()
    cache._accept((now.nanoseconds, {ship.model_name: (1., 2., 0., now.nanoseconds)}))
    node.control_loop()
    assert len(node.tracked_pub.publish.call_args.args[0].ships) == 1
    node.tracked_pub.reset_mock()

    node._gazebo_model_names.return_value = set()
    node.reconcile_gazebo_models()
    assert ship.model_name in node.ships
    # The next complete pose frame omits the removed model: old accumulated
    # control samples cannot make a fresh tracked snapshot for that model.
    now.nanoseconds += 100_000_000
    cache._accept((now.nanoseconds, {}))
    node.control_loop()
    node.tracked_pub.publish.assert_not_called()
    node.reconcile_gazebo_models()
    assert not node.ships and ship._closed
    assert not ship.publish_cmd_vel(Twist())
    assert ship.model_name not in cache._samples

    # After the removal fence, publish a fresh empty list to clear consumers.
    now.nanoseconds += 100_000_000
    cache._accept((now.nanoseconds, {}))
    node.control_loop()
    assert list(node.tracked_pub.publish.call_args.args[0].ships) == []
    assert json.loads(node.names_pub.publish.call_args.args[0].data) == []


def test_query_failure_is_not_evidence_of_deletion():
    node, ship, _, _ = make_manager()
    node._gazebo_model_names.return_value = None
    for _ in range(3):
        node.reconcile_gazebo_models()
    assert node.ships == {ship.model_name: ship}
    assert not ship._closed and not node._missing_model_counts


def test_service_delete_retires_only_after_confirmed_removal():
    node, ship, _, _ = make_manager()
    node._remove_gazebo = Mock()
    request = SimpleNamespace(model_name=ship.model_name)
    response = SimpleNamespace(success=False, message='')
    node.on_delete(request, response)
    assert response.success and not node.ships and ship._closed
    ship.cleanup.assert_called_once()


def test_failed_service_delete_keeps_registration():
    node, ship, _, _ = make_manager()
    node._remove_gazebo = Mock(side_effect=RuntimeError('not confirmed'))
    response = SimpleNamespace(success=True, message='')
    node.on_delete(SimpleNamespace(model_name=ship.model_name), response)
    assert not response.success and ship.model_name in node.ships
    assert not ship._closed
