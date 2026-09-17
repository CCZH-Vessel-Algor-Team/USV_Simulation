from types import SimpleNamespace

from usv_sim_full.scripts.storm_field_manager_node import (
    StormField,
    StormFieldManager,
)


class _ManagerFixture:
    def __init__(self, storm):
        self.storms = {storm.name: storm}
        self.parameters = {}

    def set_parameters(self, parameters):
        self.parameters = {parameter.name: parameter.value for parameter in parameters}


def test_config_changes_apply_only_to_future_storms():
    storm = StormField(
        'storm_1', 10.0, 20.0, 30.0, 0.0, 1.0, 600.0, 100.0, 0.0)
    manager = _ManagerFixture(storm)
    request = SimpleNamespace(
        radius=80.0,
        drift_heading_deg=90.0,
        drift_speed=2.0,
        weather_validity_duration_s=1200.0,
        weather_grid_resolution_m=200.0,
    )
    response = SimpleNamespace()

    StormFieldManager.on_set_config(manager, request, response)

    assert response.success
    assert storm.radius == 30.0
    assert storm.drift_heading_deg == 0.0
    assert storm.drift_speed == 1.0
    assert manager.parameters == {
        'drift_heading_deg': 90.0,
        'drift_speed': 2.0,
        'radius': 80.0,
        'weather_validity_duration_s': 1200.0,
        'weather_grid_resolution_m': 200.0,
    }
