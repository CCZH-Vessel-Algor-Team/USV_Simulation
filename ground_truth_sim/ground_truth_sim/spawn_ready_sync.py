"""Spawn-ready handshake helpers for ground_truth_node ↔ gazebo_models_node."""

from __future__ import annotations

from typing import AbstractSet, Mapping, MutableMapping, Set

DEFAULT_MOTION_READY_TOPIC = "sim/gt_motion_ready"


def normalize_motion_ready_topic(topic: str) -> str:
    """Return topic without leading slash (ROS 2 relative name)."""
    t = str(topic or "").strip() or DEFAULT_MOTION_READY_TOPIC
    if t.startswith("/"):
        t = t.lstrip("/")
    return t


def all_tracks_spawn_ready(
    expected_ids: AbstractSet[int],
    spawned_ids: AbstractSet[int],
    drivers: Mapping[int, object],
    pending_spawn_ids: AbstractSet[int],
) -> bool:
    """True when every expected track is spawned, has a driver, and is not pending create."""
    if not expected_ids:
        return False
    for tid in expected_ids:
        if tid in pending_spawn_ids:
            return False
        if tid not in spawned_ids:
            return False
        if tid not in drivers:
            return False
    return True


def pending_spawn_track_ids(spawn_futures: MutableMapping[int, object]) -> Set[int]:
    """Track ids with an in-flight create CLI future."""
    return set(spawn_futures.keys())
