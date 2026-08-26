"""Stable simulated AIS MMSI assignment shared by dynamic ships."""

import uuid


SIM_AIS_MMSI_BASE = 200000000
SIM_AIS_MMSI_SPAN = 700000000


def simulated_ais_mmsi(target_id: str) -> int:
    """Match the MMSI assigned to this target by the simulated AIS/fusion path."""
    track_id = int.from_bytes(uuid.UUID(target_id).bytes[:4], 'big')
    if track_id <= 0:
        return 0
    return SIM_AIS_MMSI_BASE + (track_id % SIM_AIS_MMSI_SPAN)
