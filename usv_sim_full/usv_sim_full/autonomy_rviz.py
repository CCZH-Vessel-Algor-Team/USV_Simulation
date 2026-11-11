"""Build the CCS v2 RViz configuration from its existing sensor/display template."""
import copy


def configure_autonomy_panel(document, namespace, simulation_tests=True):
    config = copy.deepcopy(document)
    namespace = namespace.strip('/')
    panel_class = 'nav2_rviz_plugins/USV Autonomy'
    existing = next((p for p in config.get('Panels', []) if p.get('Class') == panel_class), None)
    panels = [p for p in config.get('Panels', []) if p.get('Class') != panel_class]
    for original in panels:
        if original.get('Class') == 'nav2_rviz_plugins/Navigation 2':
            # Keep the original widget and target workflow; its optional adapter
            # routes requests through the task lifecycle instead of bypassing it.
            original['Managed Task'] = True
            original['Namespace'] = namespace
    panel = dict(existing or {})
    panel.update({'Class': panel_class, 'Namespace': namespace, 'Global Frame': 'map',
                  'Simulation Tests': bool(simulation_tests)})
    panel.setdefault('Name', 'USV BT v2')
    geometry = config.get('Window Geometry', {})
    if isinstance(geometry.get('Width'), int) and isinstance(geometry.get('Height'), int):
        panel['Base Window Width'] = geometry['Width']
        panel['Base Window Height'] = geometry['Height']
    panels.append(panel)
    config['Panels'] = panels
    manager = config.setdefault('Visualization Manager', {})
    display_name = 'HOLD Region (inner / outer)'
    displays = [d for d in manager.get('Displays', []) if d.get('Name') != display_name]
    displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': display_name,
                     'Enabled': True, 'Namespaces': {},
                     'Topic': {'Value': '/' + namespace + '/autonomy/hold_region' if namespace
                               else '/autonomy/hold_region', 'Depth': 1,
                               'Durability Policy': 'Transient Local',
                               'Reliability Policy': 'Reliable', 'History Policy': 'Keep Last'}})
    manager['Displays'] = displays
    goal_class = 'nav2_rviz_plugins/USV Goal'
    tools = [tool for tool in manager.get('Tools', []) if tool.get('Class') != goal_class]
    topic = '/' + namespace + '/autonomy/goal_pose' if namespace else '/autonomy/goal_pose'
    tools.append({'Class': goal_class, 'Topic': topic})
    manager['Tools'] = tools
    # In particular preserve QMainWindow State, camera docks, view, size and
    # all original panels/tools/displays; managed controls and HOLD display append.
    return config
