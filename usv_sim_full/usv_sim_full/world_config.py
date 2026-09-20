"""Create per-run physics settings without modifying the reference world."""

import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def world_with_real_time_factor(world_file, real_time_factor):
    """Write a runtime world with a requested simulation/wall-clock ratio.

    :param world_file: Original SDF/world path.
    :param real_time_factor: Positive finite target ratio, e.g. 1/3.
    :return: Path to a retained temporary world for launch diagnostics.
    """
    factor = float(real_time_factor)
    if not math.isfinite(factor) or factor <= 0:
        raise ValueError('real_time_factor must be finite and positive')
    source = Path(world_file)
    tree = ET.parse(source)
    world = tree.getroot().find('world')
    if world is None:
        raise ValueError(f'No world element in {source}')
    physics = world.find('physics')
    if physics is None:
        physics = ET.SubElement(world, 'physics', {'name': 'runtime_physics', 'type': 'ignored'})
    step = float(physics.findtext('max_step_size', '0.001'))
    if not math.isfinite(step) or step <= 0:
        raise ValueError('World max_step_size must be finite and positive')
    for name, value in [('real_time_factor', factor), ('real_time_update_rate', factor / step)]:
        element = physics.find(name)
        if element is None:
            element = ET.SubElement(physics, name)
        element.text = format(value, '.17g')
    # The integration step is unchanged. Original world resources remain in the
    # launch's GZ_SIM_RESOURCE_PATH; the generated file is retained for evidence.
    with tempfile.NamedTemporaryFile(prefix='usv_world_', suffix='.sdf', delete=False) as target:
        tree.write(target, encoding='utf-8', xml_declaration=True)
        return target.name


def set_navigation_clock(parameters, use_sim_time):
    """Set clocks explicitly, including Nav2 nodes absent from the input YAML.

    RewrittenYaml only substitutes existing leaf keys; passing a launch argument
    alone does not configure missing node parameters.

    :param parameters: Mutable Nav2 parameter mapping.
    :param use_sim_time: Boolean clock selection.
    """
    for name in ('controller_server', 'smoother_server', 'planner_server', 'behavior_server',
                 'bt_navigator', 'waypoint_follower', 'velocity_smoother'):
        node_params = parameters.setdefault(name, {}).setdefault('ros__parameters', {})
        node_params['use_sim_time'] = use_sim_time

    def update(mapping):
        for key, value in mapping.items():
            if not isinstance(value, dict):
                continue
            if key == 'ros__parameters':
                value['use_sim_time'] = use_sim_time
            else:
                update(value)

    update(parameters)
