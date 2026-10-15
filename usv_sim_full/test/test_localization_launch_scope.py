"""Exercise real Humble Timer/Group events without spawning any process."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import time

import pytest
from launch import Action, LaunchDescription, LaunchDescriptionSource, LaunchService
from launch.actions import (
    GroupAction, IncludeLaunchDescription, OpaqueFunction, SetLaunchConfiguration,
)
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.utilities import evaluate_parameters, normalize_parameters


ROOT = Path(__file__).parents[1]


@pytest.fixture
def robot_launch(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path))
    spec = importlib.util.spec_from_file_location(
        'robot_scope_test', ROOT / 'launch/components/robot_bringup.launch.py',
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    probes = []

    class ProbeNode(Action):
        def __init__(self, **kwargs):
            super().__init__(condition=kwargs.get('condition'))
            self.kwargs = kwargs

        def execute(self, context):
            if self.kwargs['executable'] not in ('navsat_transform_node', 'ekf_node'):
                return
            native = Node(**self.kwargs)
            native._perform_substitutions(context)
            probes.append({
                'executable': self.kwargs['executable'],
                'namespace': native.expanded_node_namespace,
                'parameters': evaluate_parameters(
                    context, normalize_parameters(self.kwargs['parameters'])),
                'global_params': deepcopy(context.launch_configurations.get('global_params', [])),
                'remaps': list(native.expanded_remapping_rules or []),
                'time': time.monotonic(),
            })

    real_opaque = module.OpaqueFunction

    def safe_opaque(*, function):
        if function.__name__ in ('launch_robot_state_publisher',
                                 'launch_spawn_robot_when_ready', 'launch_obstacle_spawner'):
            return real_opaque(function=lambda context: [])
        return real_opaque(function=function)

    monkeypatch.setattr(module, 'Node', ProbeNode)
    monkeypatch.setattr(module, 'OpaqueFunction', safe_opaque)
    return module, probes


def run(module, arguments, sibling_actions=(), parent='fleet'):
    popped = {}

    def mark_pop(context):
        popped.update(
            time=time.monotonic(), configurations=deepcopy(context.launch_configurations),
        )

    include = IncludeLaunchDescription(
        LaunchDescriptionSource(module.generate_launch_description()),
        launch_arguments=arguments.items(),
    )
    service = LaunchService(noninteractive=True)
    service.include_launch_description(LaunchDescription([
        GroupAction(scoped=True, actions=[
            PushRosNamespace(parent),
            SetParameter('registration_marker', 'robot'),
            SetRemap('/tf', f'/{parent}/{arguments["robot_name"]}/tf'),
            include,
            SetParameter('late_mutation', 'not_for_localization'),
        ]),
        *sibling_actions,
        OpaqueFunction(function=mark_pop),
    ]))
    assert service.run() == 0
    assert service.context.launch_configurations == popped['configurations']
    return popped


def arguments(tmp_path, **overrides):
    params = tmp_path / 'localization.yaml'
    params.write_text('/**:\n  ros__parameters:\n    frequency: 20.0\n', encoding='utf-8')
    return {
        'robot_name': 'boat', 'enable_robot_localization': 'true',
        'external_localization': 'false', 'localization_start_delay': '0.05',
        'localization_params_file': str(params), 'use_sim_time': 'true',
        'enable_obstacle_spawner': 'false', 'enable_maritime_radar_bridge': 'false',
        **overrides,
    }


def test_enabled_localization_survives_scope_pop_and_root_overwrites(robot_launch, tmp_path):
    module, probes = robot_launch
    args = arguments(tmp_path)
    root = {
        'robot_name': 'wrong_robot', 'enable_robot_localization': 'false',
        'external_localization': 'true', 'localization_start_delay': 'invalid_root_delay',
        'localization_params_file': '/wrong/file.yaml', 'use_sim_time': 'false',
        'ros_namespace': '/wrong_namespace',
    }
    popped = run(module, args, [SetLaunchConfiguration(key, value) for key, value in root.items()])
    assert [probe['executable'] for probe in probes] == ['navsat_transform_node', 'ekf_node']
    for probe in probes:
        assert probe['time'] > popped['time']
        assert probe['namespace'] == '/fleet/boat'
        assert probe['parameters'][0] == Path(args['localization_params_file'])
        assert probe['parameters'][1]['use_sim_time'] is True
        assert probe['global_params'] == [('registration_marker', 'robot')]
        assert ('/tf', '/fleet/boat/tf') in probe['remaps']


@pytest.mark.parametrize('external,enabled', [
    ('true', 'true'), ('true', 'false'), ('false', 'false'),
])
def test_disabled_or_external_localization_creates_no_delayed_nodes(
        robot_launch, tmp_path, external, enabled):
    module, probes = robot_launch
    run(module, arguments(tmp_path, external_localization=external,
                          enable_robot_localization=enabled,
                          localization_start_delay='not_evaluated_when_disabled'))
    assert probes == []


@pytest.mark.parametrize('delay', ['-1', 'nan', 'inf'])
def test_invalid_enabled_delay_fails_at_registration(robot_launch, tmp_path, delay):
    module, _ = robot_launch
    description = module.generate_launch_description()
    schedule = next(action for action in description.entities
                    if isinstance(action, GroupAction)).get_sub_entities()
    service = LaunchService(noninteractive=True)
    service.context.launch_configurations.update(arguments(
        tmp_path, localization_start_delay=delay,
    ))
    with pytest.raises(ValueError, match='finite and nonnegative'):
        for action in schedule:
            if isinstance(action, OpaqueFunction):
                action.execute(service.context)
