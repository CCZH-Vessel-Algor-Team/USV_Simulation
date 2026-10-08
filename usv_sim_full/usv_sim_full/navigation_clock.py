"""Apply the selected navigation clock after merging node parameter files."""


def set_navigation_clock(parameters, use_sim_time):
    """Set clocks explicitly, including nodes absent from the input YAML.

    RewrittenYaml only substitutes existing keys; passing a launch argument alone
    does not configure node parameters that are missing from the merged mapping.

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
