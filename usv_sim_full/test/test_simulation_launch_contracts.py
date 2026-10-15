"""Runtime model export and launch options without running any node or process."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction


PACKAGE = Path(__file__).resolve().parents[1]


def _load_launch(name, relative_path):
    spec = importlib.util.spec_from_file_location(name, PACKAGE / relative_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **overrides):
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    context.launch_configurations.update(overrides)
    return context


@pytest.fixture
def main_launch(monkeypatch, tmp_path):
    module = _load_launch('main_runtime_contract', 'launch/main.launch.py')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(PACKAGE))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: SimpleNamespace(**kwargs))
    monkeypatch.setattr(
        module, 'session_manager_executable_path', lambda: '/not-executed/session_manager',
    )
    monkeypatch.setattr(module, 'load_mmwave_sensor_defaults', lambda *args: {})
    monkeypatch.setattr(module, 'load_mmwave_cluster_defaults', lambda *args: {})
    session_dir = tmp_path / 'generated session'
    session_dir.mkdir()
    robots = []
    for name in ('usv_1', 'usv_2'):
        urdf = session_dir / f'{name}.urdf'
        urdf.write_text(
            f'<robot name="{name}"><mesh filename="model://hull/mesh/hull.dae"/></robot>',
        )
        bridge = session_dir / f'{name}_bridge.yaml'
        bridge.write_text(yaml.safe_dump([
            {'ros_topic_name': f'/model/{name}/joint_state',
             'gz_topic_name': f'/world/test/model/{name}/joint_state', 'direction': 'GZ_TO_ROS'},
            {'ros_topic_name': f'/{name}/thrusters/left/thrust',
             'gz_topic_name': f'/{name}/thrusters/left/thrust', 'direction': 'ROS_TO_GZ'},
        ]))
        robots.append({'name': name, 'urdf_path': str(urdf), 'bridge_yaml_path': str(bridge)})
    rviz = session_dir / 'session.rviz'
    rviz.write_text('Visualization Manager:\n  Displays: []\n')
    session = {
        'robots': robots, 'session_path': str(session_dir),
        'rviz_config_path': str(rviz),
        'obstacle_layout_path': str(session_dir / 'obstacles.json'),
    }

    def session_manager(command, **kwargs):
        assert command[0] == '/not-executed/session_manager'
        assert kwargs == {'capture_output': True, 'text': True, 'check': True}
        return SimpleNamespace(stdout='Session generated\n' + json.dumps(session))

    monkeypatch.setattr(module.subprocess, 'run', session_manager)
    config = {'visualization': {'launch_rviz': False},
              'robot_1': {'name': 'usv_1'}, 'robot_2': {'name': 'usv_2'}}
    config_path = tmp_path / 'full_config.yaml'
    config_path.write_text(yaml.safe_dump(config))
    context = _context(module, config_path=str(config_path), enable_dynamic_buoy_manager='false')
    return SimpleNamespace(
        module=module, context=context, config=config, config_path=config_path,
        session=session, manifest=tmp_path / 'runtime output' / 'robot model.yaml',
    )


def _expected_manifest(robots):
    return {
        'schema_version': 1,
        'kind': 'robot_model_manifest',
        'robots': [
            {'name': robot['name'], 'urdf_path': os.path.abspath(robot['urdf_path']),
             'joint_states_topic': f'/model/{robot["name"]}/joint_state'}
            for robot in robots
        ],
    }


def test_optional_main_arguments_default_to_empty(main_launch):
    assert main_launch.context.launch_configurations['model_manifest_file'] == ''
    assert main_launch.context.launch_configurations['launch_rviz'] == ''


def test_manifest_is_atomic_and_complete_before_node_construction(
        main_launch, monkeypatch, tmp_path):
    module, context, manifest = main_launch.module, main_launch.context, main_launch.manifest
    manifest.parent.mkdir()
    stale = b'stale: [invalid yaml'
    manifest.write_bytes(stale)
    inputs = {
        Path(robot[key]): Path(robot[key]).read_bytes()
        for robot in main_launch.session['robots'] for key in ('urdf_path', 'bridge_yaml_path')
    }
    monkeypatch.chdir(tmp_path)
    main_launch.session['robots'][1]['urdf_path'] = os.path.relpath(
        main_launch.session['robots'][1]['urdf_path'], tmp_path,
    )
    resolved = []
    resolve = module.resolve_session_robots

    def resolve_robots(session, config):
        robots = list(reversed(resolve(session, config)))
        resolved.extend(robots)
        return robots

    monkeypatch.setattr(module, 'resolve_session_robots', resolve_robots)
    replacements = []
    replace = module.os.replace

    def atomic_replace(source, destination):
        assert resolved
        assert Path(source).parent == manifest.parent
        assert Path(source) != Path(destination) == manifest
        assert manifest.read_bytes() == stale
        assert yaml.safe_load(Path(source).read_text()) == _expected_manifest(resolved)
        replace(source, destination)
        replacements.append((source, destination))

    monkeypatch.setattr(module.os, 'replace', atomic_replace)
    nodes = []

    def node(**kwargs):
        assert yaml.safe_load(manifest.read_text()) == _expected_manifest(resolved)
        nodes.append(kwargs)
        return SimpleNamespace(**kwargs)

    monkeypatch.setattr(module, 'Node', node)
    context.launch_configurations['model_manifest_file'] = os.path.relpath(manifest, tmp_path)
    items = module.launch_setup(context)
    assert items and nodes and len(replacements) == 1
    assert yaml.safe_load(manifest.read_text()) == _expected_manifest(resolved)
    assert list(manifest.parent.iterdir()) == [manifest]
    assert all(path.read_bytes() == original for path, original in inputs.items())
    includes = [item for item in items if isinstance(item, IncludeLaunchDescription)
                and 'robot_name' in dict(item.launch_arguments)]
    assert [dict(item.launch_arguments)['urdf_path'] for item in includes] == [
        robot['urdf_path'] for robot in resolved
    ]


def test_manifest_export_creates_missing_runtime_directory(main_launch):
    assert not main_launch.manifest.parent.exists()
    main_launch.context.launch_configurations['model_manifest_file'] = str(main_launch.manifest)
    main_launch.module.launch_setup(main_launch.context)
    assert yaml.safe_load(main_launch.manifest.read_text()) == _expected_manifest(
        main_launch.session['robots'],
    )


def test_empty_manifest_path_does_not_export_or_use_an_old_manifest(main_launch, monkeypatch):
    main_launch.manifest.parent.mkdir()
    main_launch.manifest.write_text('stale: [invalid yaml')
    monkeypatch.setattr(
        main_launch.module.os, 'replace', lambda *args: pytest.fail('unexpected export'),
    )
    monkeypatch.setattr(main_launch.module.tempfile, 'NamedTemporaryFile',
                        lambda **kwargs: pytest.fail('unexpected temporary manifest'))
    assert main_launch.module.launch_setup(main_launch.context)
    assert main_launch.manifest.read_text() == 'stale: [invalid yaml'


@pytest.mark.parametrize('stage', ['directory', 'temporary_file', 'write', 'replace', 'payload'])
def test_manifest_export_failure_aborts_without_stale_fallback(main_launch, monkeypatch, stage):
    module, context, manifest = main_launch.module, main_launch.context, main_launch.manifest
    manifest.parent.mkdir()
    manifest.write_text('stale: [invalid yaml')
    context.launch_configurations['model_manifest_file'] = str(manifest)
    monkeypatch.setattr(
        module, 'Node', lambda **kwargs: pytest.fail('node constructed after export failure'),
    )

    def fail(*args, **kwargs):
        if stage == 'write':
            args[1].write('partial: [document')
        raise OSError(f'injected {stage} failure')

    if stage == 'directory':
        monkeypatch.setattr(module.os, 'makedirs', fail)
    elif stage == 'temporary_file':
        monkeypatch.setattr(module.tempfile, 'NamedTemporaryFile', fail)
    elif stage == 'write':
        monkeypatch.setattr(module.yaml, 'safe_dump', fail)
    elif stage == 'replace':
        monkeypatch.setattr(module.os, 'replace', fail)
    else:
        main_launch.session['robots'][0]['urdf_path'] = None
    with pytest.raises(RuntimeError, match='Failed to export robot model manifest') as error:
        module.launch_setup(context)
    assert str(manifest) in str(error.value)
    assert isinstance(error.value.__cause__, TypeError if stage == 'payload' else OSError)
    assert manifest.read_text() == 'stale: [invalid yaml'
    assert list(manifest.parent.iterdir()) == [manifest]


def test_manifest_exports_resolved_single_robot_session(main_launch):
    robot = main_launch.session.pop('robots')[0]
    main_launch.session.update(
        urdf_path=robot['urdf_path'], bridge_yaml_path=robot['bridge_yaml_path'],
    )
    config = {'robot': {'name': robot['name']}, 'visualization': {'launch_rviz': False}}
    main_launch.config_path.write_text(yaml.safe_dump(config))
    main_launch.context.launch_configurations['model_manifest_file'] = str(main_launch.manifest)
    main_launch.module.launch_setup(main_launch.context)
    assert yaml.safe_load(main_launch.manifest.read_text()) == _expected_manifest([robot])


@pytest.mark.parametrize('yaml_rviz', [True, False, None])
@pytest.mark.parametrize('override', ['', 'true', 'false'])
@pytest.mark.parametrize('headless', ['false', 'true'])
def test_rviz_override_is_independent_of_headless(main_launch, yaml_rviz, override, headless):
    if yaml_rviz is None:
        main_launch.config['visualization'].pop('launch_rviz')
    else:
        main_launch.config['visualization']['launch_rviz'] = yaml_rviz
    main_launch.config_path.write_text(yaml.safe_dump(main_launch.config))
    main_launch.context.launch_configurations.update(launch_rviz=override, gz_headless=headless)
    items = main_launch.module.launch_setup(main_launch.context)
    includes = [dict(item.launch_arguments) for item in items
                if isinstance(item, IncludeLaunchDescription)]
    rviz = [arguments for arguments in includes if 'rviz_config_path' in arguments]
    expected = override == 'true' if override else yaml_rviz is not False
    assert len(rviz) == int(expected)
    infra = next(arguments for arguments in includes if 'world_name' in arguments)
    assert infra['gz_headless'] == headless


@pytest.mark.parametrize('override,later_value', [('true', 'false'), ('false', 'true')])
def test_rviz_override_uses_frozen_runtime_value(main_launch, monkeypatch, override, later_value):
    context = main_launch.context
    context.launch_configurations['launch_rviz'] = override
    run_session_manager = main_launch.module.subprocess.run

    def session_manager(*args, **kwargs):
        context.launch_configurations['launch_rviz'] = later_value
        return run_session_manager(*args, **kwargs)

    monkeypatch.setattr(main_launch.module.subprocess, 'run', session_manager)
    items = main_launch.module.launch_setup(context)
    rviz = [item for item in items if isinstance(item, IncludeLaunchDescription)
            and 'rviz_config_path' in dict(item.launch_arguments)]
    assert len(rviz) == int(override == 'true')


@pytest.mark.parametrize('headless', [None, 'false', 'true', '0', '1', 'yes'])
def test_headless_cli_uses_egl_and_leaves_default_gui_unchanged(monkeypatch, tmp_path, headless):
    module = _load_launch('infra_runtime_contract', 'launch/components/infra_sim.launch.py')
    callbacks = {}

    def capture_callback(*, function):
        callbacks[function.__name__] = function
        return OpaqueFunction(function=function)

    monkeypatch.setattr(module, 'OpaqueFunction', capture_callback)
    share = tmp_path / 'simulation share'
    worlds = share / 'worlds'
    worlds.mkdir(parents=True)
    world = worlds / 'sydney_regatta.sdf'
    world.write_text('<sdf version="1.9"><world name="sydney_regatta"/></sdf>')
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(share))
    monkeypatch.setattr(module, 'get_package_prefix', lambda package: str(tmp_path / package))
    monkeypatch.setattr(module, 'ExecuteProcess', lambda **kwargs: SimpleNamespace(**kwargs))
    context = _context(module, **({} if headless is None else {'gz_headless': headless}))
    if headless is None:
        assert context.launch_configurations['gz_headless'] == 'false'
    processes = callbacks['launch_gazebo_with_selected_world'](context)
    offscreen = headless in ('true', '1', 'yes')
    assert len(processes) == (1 if offscreen else 2)
    commands = []
    for process in processes:
        assert process.cmd[:2] == ['bash', '-lc']
        commands.append(shlex.split(process.cmd[2].split('2>&1 |', 1)[0]))
    server_args = ['-r', '-s', str(world)] + (['--headless-rendering'] if offscreen else [])
    assert commands[0] == ['ruby', '$(which gz)', 'sim', *server_args, '--force-version', '8']
    if not offscreen:
        assert commands[1] == ['ruby', '$(which gz)', 'sim', '-g', '--force-version', '8']
