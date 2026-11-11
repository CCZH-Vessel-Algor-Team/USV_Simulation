"""CCS rotates stores only at the whole-system boundary, without launching Gazebo."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from usv_sim_full.autonomy_session import new_autonomy_session, autonomy_session_paths


ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ccs_context(root, **overrides):
    context = LaunchContext()
    context.launch_configurations.update({
        'autonomy_v2': 'true', 'enable_autonomy_test_controls': 'true',
        'nav2_namespace': 'test_boat', 'config_path': 'not-read-with-explicit-namespace.yaml',
        'rviz_config_path': str(ROOT / 'rviz/three_vision_one_mmwave.rviz'),
        'use_sim_time': 'true', 'nav2_min_wait_sec': '8.0',
        'nav2_readiness_timeout_sec': '180.0', 'enable_nav2': 'true',
        'monitor_publish_frequency': '5.0', 'ccs_session_root': str(root), **overrides,
    })
    return context


def included_arguments(actions):
    return dict(next(a for a in actions if isinstance(a, IncludeLaunchDescription)).launch_arguments)


def test_whole_ccs_restart_is_fresh_but_reentry_and_other_vessels_reuse_session(tmp_path, monkeypatch):
    module = load(ROOT / 'launch/CCS_Certified_Simulation_Environment.launch.py', 'ccs_session_contract')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    temporary = module.tempfile.NamedTemporaryFile
    monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', lambda **kw: temporary(dir=tmp_path, **kw))
    root = tmp_path / 'sessions'
    first_context = ccs_context(root)
    first = included_arguments(module._ccs_base_bringup(first_context))
    database = Path(first['autonomy_database'])
    checkpoint = Path(first['autonomy_rules_checkpoint'])
    database.write_bytes(b'old task database must remain untouched')
    checkpoint.write_text('{"waiting_targets": ["old-target"], "event_id": "old-handover"}')
    same = included_arguments(module._ccs_base_bringup(first_context))
    assert same['autonomy_database'] == str(database)
    assert same['autonomy_rules_checkpoint'] == str(checkpoint)
    first_context.launch_configurations['nav2_namespace'] = 'other_boat'
    other = included_arguments(module._ccs_base_bringup(first_context))
    assert Path(other['autonomy_database']).parent.parent == database.parent.parent
    assert other['autonomy_database'] != str(database)
    second = included_arguments(module._ccs_base_bringup(ccs_context(root)))
    assert second['autonomy_database'] != str(database)
    assert second['autonomy_rules_checkpoint'] != str(checkpoint)
    assert not Path(second['autonomy_database']).exists()
    assert not Path(second['autonomy_rules_checkpoint']).exists()
    assert database.read_bytes() == b'old task database must remain untouched'
    assert 'old-handover' in checkpoint.read_text()


def test_non_v2_ccs_does_not_create_an_autonomy_session(tmp_path, monkeypatch):
    module = load(ROOT / 'launch/CCS_Certified_Simulation_Environment.launch.py', 'ccs_non_v2_session')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    root = tmp_path / 'sessions'
    arguments = included_arguments(module._ccs_base_bringup(ccs_context(root, autonomy_v2='false')))
    assert 'autonomy_database' not in arguments and 'autonomy_rules_checkpoint' not in arguments
    assert not root.exists()


@pytest.mark.parametrize('namespace', ['../escape', 'fleet/../escape', '', 'bad-name'])
def test_session_paths_reject_namespace_traversal(tmp_path, namespace):
    with pytest.raises(ValueError, match='namespace'):
        autonomy_session_paths(tmp_path, namespace)


def test_nested_vessel_namespaces_stay_inside_a_unique_session(tmp_path):
    session = new_autonomy_session(tmp_path)
    paths = autonomy_session_paths(session, '/fleet/boat/')
    assert Path(paths['autonomy_database']).parent == session / 'fleet/boat'
    assert autonomy_session_paths(session, 'fleet/boat') == paths


def test_invalid_session_root_fails_without_falling_back_to_old_stores(tmp_path):
    root = tmp_path / 'not-a-directory'
    root.write_text('do not overwrite')
    with pytest.raises(OSError):
        new_autonomy_session(root)
    assert root.read_text() == 'do not overwrite'
    with pytest.raises(ValueError, match='must not be empty'):
        new_autonomy_session('')


def capture_launch(module, monkeypatch):
    callbacks, includes = {}, []
    opaque, include = module.OpaqueFunction, module.IncludeLaunchDescription
    def capture_opaque(*, function):
        callbacks[function.__name__] = function
        return opaque(function=function)
    def capture_include(*args, **kwargs):
        includes.append(dict(kwargs.get('launch_arguments', [])))
        return include(*args, **kwargs)
    monkeypatch.setattr(module, 'OpaqueFunction', capture_opaque)
    monkeypatch.setattr(module, 'IncludeLaunchDescription', capture_include)
    description = module.generate_launch_description()
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context, callbacks, includes


def test_paths_reach_deferred_navigation_and_both_node_parameters(tmp_path, monkeypatch):
    paths = autonomy_session_paths(new_autonomy_session(tmp_path / 'sessions'), 'test_boat')
    simulation = load(ROOT / 'launch/nav2_sim_three_vision_mmwave_bringup.launch.py', 'session_sim_forwarding')
    monkeypatch.setattr(simulation, 'get_package_share_directory', lambda _: str(ROOT))
    context, callbacks, includes = capture_launch(simulation, monkeypatch)
    context.launch_configurations.update(paths, nav2_namespace='test_boat', autonomy_v2='true')
    # Call only readiness setup. Never execute prelaunch_cleanup or any process.
    callbacks['nav2_readiness_and_deferred_launch'](context)
    forwarded = next(args for args in includes if 'autonomy_database' in args)
    assert all(forwarded[key] == value for key, value in paths.items())
    assert not Path(paths['autonomy_database']).exists()

    thruster = load(ROOT / 'launch/nav2_thruster_bringup.launch.py', 'session_thruster_forwarding')
    monkeypatch.setattr(thruster, 'get_package_share_directory', lambda _: str(ROOT))
    temporary = thruster.tempfile.NamedTemporaryFile
    monkeypatch.setattr(thruster.tempfile, 'NamedTemporaryFile', lambda **kw: temporary(dir=tmp_path, **kw))
    context, callbacks, includes = capture_launch(thruster, monkeypatch)
    context.launch_configurations.update(paths, namespace='test_boat', autonomy_v2='true',
                                         control_params_file='')
    callbacks['_launch_nav2_with_namespaced_map'](context)
    leaf = next(args for args in includes if 'rules_checkpoint' in args)
    assert leaf['database'] == paths['autonomy_database']
    assert leaf['rules_checkpoint'] == paths['autonomy_rules_checkpoint']
    # Recreating a child include does not create a new session or overwrite data.
    Path(paths['autonomy_rules_checkpoint']).write_text('retained risk')
    callbacks['_launch_nav2_with_namespaced_map'](context)
    assert includes[-1]['rules_checkpoint'] == leaf['rules_checkpoint']
    assert Path(leaf['rules_checkpoint']).read_text() == 'retained risk'

    autonomy = load(ROOT.parents[1] / 'usv_nav/usv_autonomy/launch/autonomy.launch.py', 'session_leaf_parameters')
    monkeypatch.setattr(autonomy, 'get_package_share_directory', lambda _: str(ROOT))
    nodes = []
    def node(**kwargs):
        nodes.append(kwargs)
        return SimpleNamespace(**kwargs)
    monkeypatch.setattr(autonomy, 'Node', node)
    description = autonomy.generate_launch_description()
    context = LaunchContext()
    from launch.actions import OpaqueFunction
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(database=leaf['database'], rules_checkpoint=leaf['rules_checkpoint'],
                                         launch_nav2='false', authority_adapter='false')
    next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    mission = next(n for n in nodes if n['executable'] == 'mission')
    rules = next(n for n in nodes if n['executable'] == 'maritime_rules')
    assert mission['parameters'][-1]['database'] == paths['autonomy_database']
    assert rules['parameters'][-1]['rules_checkpoint'] == paths['autonomy_rules_checkpoint']


def test_partial_path_pair_is_rejected_instead_of_mixing_sessions(tmp_path, monkeypatch):
    module = load(ROOT / 'launch/nav2_thruster_bringup.launch.py', 'session_incomplete_paths')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    temporary = module.tempfile.NamedTemporaryFile
    monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', lambda **kw: temporary(dir=tmp_path, **kw))
    context, callbacks, _ = capture_launch(module, monkeypatch)
    context.launch_configurations.update(autonomy_v2='true', control_params_file='',
                                         autonomy_database=str(tmp_path / 'tasks.db'))
    with pytest.raises(ValueError, match='supplied together'):
        callbacks['_launch_nav2_with_namespaced_map'](context)
