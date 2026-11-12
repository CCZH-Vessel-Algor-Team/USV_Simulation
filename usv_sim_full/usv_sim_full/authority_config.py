"""One authority source per vessel. This does not select the actuator backend."""


def authority_launch_arguments(source, test_controls, topic='/usv1/cc/state', clock='system'):
    if source not in ('cc_state', 'simulation'):
        raise ValueError('authority_source must be cc_state or simulation')
    if clock not in ('system', 'ros'):
        raise ValueError('fcu_state_clock must be system or ros')
    if str(test_controls).lower() not in ('true', 'false'):
        raise ValueError('test_controls must be true or false')
    tests = str(test_controls).lower() == 'true'
    if source == 'cc_state' and not topic.startswith('/'):
        raise ValueError('fcu_state_topic must be absolute to bridge FCU and BT vessel namespaces')
    return {
        'authority_adapter': 'true' if source == 'cc_state' else 'false',
        'simulation_test_controls': 'true' if tests else 'false',
        'simulation_authority': 'true' if source == 'simulation' and tests else 'false',
        'fcu_state_topic': topic,
        'fcu_state_clock': clock,
    }
