"""Source selection never mixes synthetic and observed FCU authority."""
import pytest
from usv_sim_full.authority_config import authority_launch_arguments


def test_real_source_keeps_namespaces_separate_and_disables_simulated_authority():
    args = authority_launch_arguments('cc_state', 'false', '/usv1/cc/state', 'system')
    assert args == dict(authority_adapter='true', simulation_test_controls='false', simulation_authority='false',
                        fcu_state_topic='/usv1/cc/state', fcu_state_clock='system')


def test_simulated_authority_is_explicit_and_exclusive():
    args = authority_launch_arguments('simulation', 'true')
    assert args['authority_adapter'] == 'false'
    assert args['simulation_test_controls'] == 'true'
    assert args['simulation_authority'] == 'true'


def test_fault_injection_can_coexist_with_real_authority_without_simulating_it():
    args = authority_launch_arguments('cc_state', 'true')
    assert args['authority_adapter'] == 'true'
    assert args['simulation_test_controls'] == 'true'
    assert args['simulation_authority'] == 'false'


@pytest.mark.parametrize('source,tests,topic,clock', [
    ('cc_state', 'unknown', '/usv1/cc/state', 'system'),
    ('unknown', 'false', '/usv1/cc/state', 'system'),
    ('cc_state', 'false', 'cc/state', 'system'),
    ('cc_state', 'false', '/usv1/cc/state', 'unknown'),
])
def test_invalid_or_competing_configuration_fails_closed(source, tests, topic, clock):
    with pytest.raises(ValueError):
        authority_launch_arguments(source, tests, topic, clock)
