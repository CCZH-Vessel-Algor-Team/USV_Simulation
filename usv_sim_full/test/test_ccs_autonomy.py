"""Verify the real CCS RViz source and the parent-to-navigation contract."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import pytest

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
import yaml
from usv_sim_full.autonomy_rviz import configure_autonomy_panel

ROOT = Path(__file__).resolve().parents[1]


def load_ccs():
    path = ROOT / 'launch/CCS_Certified_Simulation_Environment.launch.py'
    spec = importlib.util.spec_from_file_location('ccs_autonomy_under_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_real_three_sensor_template_keeps_displays_and_adds_managed_controls():
    original = yaml.safe_load((ROOT / 'rviz/three_vision_one_mmwave.rviz').read_text())
    before = copy.deepcopy(original)
    result = configure_autonomy_panel(original, 'test_boat')
    assert original == before
    displays = result['Visualization Manager']['Displays']
    assert displays[:-1] == original['Visualization Manager']['Displays']
    assert displays[-1]['Class'] == 'rviz_default_plugins/MarkerArray'
    assert displays[-1]['Topic']['Value'] == '/test_boat/autonomy/hold_region'
    assert displays[-1]['Topic']['Durability Policy'] == 'Transient Local'
    panels = result['Panels']
    panel = next(p for p in panels if p['Class'] == 'nav2_rviz_plugins/USV Autonomy')
    assert panel['Namespace'] == 'test_boat' and panel['Simulation Tests']
    legacy = next(p for p in panels if p['Class'] == 'nav2_rviz_plugins/Navigation 2')
    assert legacy['Managed Task'] is True and legacy['Namespace'] == 'test_boat'
    for original_panel in original['Panels']:
        retained = next(p for p in panels if p['Class'] == original_panel['Class'])
        assert all(retained.get(k) == v for k, v in original_panel.items())
    tools = result['Visualization Manager']['Tools']
    assert {'Class': 'nav2_rviz_plugins/USV Goal', 'Topic': '/test_boat/autonomy/goal_pose'} in tools
    assert all(t in tools for t in original['Visualization Manager']['Tools'])
    assert result['Window Geometry'] == original['Window Geometry']
    assert result['Visualization Manager']['Views'] == original['Visualization Manager']['Views']


def test_rviz_generation_is_idempotent_and_can_hide_simulation_controls():
    original = yaml.safe_load((ROOT / 'rviz/three_vision_one_mmwave.rviz').read_text())
    result = configure_autonomy_panel(original, 'usv_1', False)
    assert configure_autonomy_panel(result, 'usv_1', False) == result
    assert next(p for p in result['Panels'] if p['Class'] == 'nav2_rviz_plugins/USV Autonomy')['Simulation Tests'] is False
    result['Window Geometry']['QMainWindow State'] = 'saved-user-layout'
    assert configure_autonomy_panel(result, 'usv_1', False)['Window Geometry']['QMainWindow State'] == 'saved-user-layout'


@pytest.mark.parametrize('stand_on', ['false', 'true'])
def test_ccs_actual_include_wires_v2_namespace_rviz_and_watchdog_frequency(tmp_path, monkeypatch, stand_on):
    module = load_ccs()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    real_temporary = tempfile.NamedTemporaryFile
    monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', lambda **kw: real_temporary(dir=tmp_path, **kw))
    context = LaunchContext()
    context.launch_configurations.update({
        'autonomy_v2': 'true', 'enable_autonomy_test_controls': 'true',
        'nav2_namespace': 'test_boat', 'config_path': 'unused-explicit-namespace.yaml',
        'rviz_config_path': str(ROOT / 'rviz/three_vision_one_mmwave.rviz'),
        'use_sim_time': 'true', 'nav2_min_wait_sec': '8.0',
        'nav2_readiness_timeout_sec': '180.0', 'enable_nav2': 'true',
        'monitor_publish_frequency': '5.0',
        'left_crossing_stand_on': stand_on,
        'ccs_session_root': str(tmp_path / 'sessions'),
    })
    actions = module._ccs_base_bringup(context)
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    arguments = dict(include.launch_arguments)
    assert arguments['autonomy_v2'] == 'true'
    assert arguments['enable_autonomy_test_controls'] == 'true'
    assert arguments['nav2_namespace'] == 'test_boat'
    assert arguments['monitor_publish_frequency'] == '5.0'
    assert arguments['left_crossing_stand_on'] == stand_on
    assert arguments['use_static_map_odom_tf'] == 'false'
    database = Path(arguments['autonomy_database'])
    checkpoint = Path(arguments['autonomy_rules_checkpoint'])
    assert database.parent == checkpoint.parent
    assert database.parent.name == 'test_boat'
    assert database.parent.parent.parent == tmp_path / 'sessions'
    assert not database.exists() and not checkpoint.exists()
    loaded = yaml.safe_load(Path(arguments['rviz_config_path']).read_text())
    assert next(p for p in loaded['Panels'] if p['Class'] == 'nav2_rviz_plugins/USV Autonomy')['Namespace'] == 'test_boat'


def test_ccs_default_entry_selects_v2_and_the_existing_sensor_template(monkeypatch):
    module = load_ccs()
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(ROOT))
    called = []
    def base(context):
        called.append(context)
        return []
    monkeypatch.setattr(module, '_ccs_base_bringup', base)
    description = module.generate_launch_description()
    arguments = {a.name: a for a in description.entities if isinstance(a, DeclareLaunchArgument)}
    context = LaunchContext()
    for name in ('autonomy_v2', 'enable_autonomy_test_controls', 'monitor_publish_frequency', 'rviz_config_path'):
        arguments[name].execute(context)
    assert context.launch_configurations['autonomy_v2'] == 'true'
    assert context.launch_configurations['enable_autonomy_test_controls'] == 'true'
    assert context.launch_configurations['monitor_publish_frequency'] == '5.0'
    assert context.launch_configurations['rviz_config_path'] == str(ROOT / 'rviz/three_vision_one_mmwave.rviz')
    next(a for a in description.entities if isinstance(a, OpaqueFunction)).execute(context)
    assert called == [context]
