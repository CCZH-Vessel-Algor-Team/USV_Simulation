"""Use Gazebo's compact scene inventory to confirm the main vessel."""
from unittest.mock import Mock

import pytest


pytest.importorskip('rclpy')
Empty = pytest.importorskip('gz.msgs10.empty_pb2').Empty
Scene = pytest.importorskip('gz.msgs10.scene_pb2').Scene

from usv_sim_full.scripts.gz_spawn_robot_when_ready import _scene_has_root_model


@pytest.mark.parametrize('ok, root_names, nested_names, expected', [
    (True, ['CCS Offshore PV Farm', 'usv_1'], [], True),
    (True, ['usv_10'], ['usv_1'], False),
    (False, ['usv_1'], [], False),
])
def test_exact_root_model_from_selected_scene(ok, root_names, nested_names, expected):
    msg = Scene()
    for name in root_names:
        msg.model.add(name=name)
    for name in nested_names:
        msg.model[0].model.add(name=name)
    node = Mock()
    node.request.return_value = (ok, msg)
    assert _scene_has_root_model(node, 'ccs_open_water_crop', 'usv_1') is expected
    node.request.assert_called_once_with('/world/ccs_open_water_crop/scene/info', Empty(), Empty,
                                        Scene, 2000)
