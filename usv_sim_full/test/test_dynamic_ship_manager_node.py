"""dynamic_ship_manager_node 的稳定 AIS MMSI 映射测试。"""

import uuid

from usv_sim_full.ais_mmsi import (
    SIM_AIS_MMSI_BASE,
    SIM_AIS_MMSI_SPAN,
    simulated_ais_mmsi,
)


def test_simulated_ais_mmsi_matches_track_id_rule():
    target_id = "12345678-9abc-def0-1234-56789abcdef0"
    expected_track_id = 0x12345678

    assert simulated_ais_mmsi(target_id) == (
        SIM_AIS_MMSI_BASE + (expected_track_id % SIM_AIS_MMSI_SPAN)
    )


def test_simulated_ais_mmsi_is_stable_for_target_id():
    target_id = str(uuid.uuid5(uuid.NAMESPACE_DNS, "dyn_target_1"))

    assert simulated_ais_mmsi(target_id) == simulated_ais_mmsi(target_id)


def test_simulated_ais_mmsi_returns_zero_for_zero_track_id():
    assert simulated_ais_mmsi("00000000-0000-0000-0000-000000000001") == 0
