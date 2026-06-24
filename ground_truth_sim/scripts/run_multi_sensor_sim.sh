#!/usr/bin/env bash
# Launches ground truth + sensor sim pipeline after building.
# Usage: ./scripts/run_multi_sensor_sim.sh [additional ros2 launch args]

set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "[run_multi_sensor_sim] Building usv_interfaces, ground_truth_sim, ground_truth_sensor_sim..."
colcon build --packages-select usv_interfaces ground_truth_sim ground_truth_sensor_sim --symlink-install

echo "[run_multi_sensor_sim] Sourcing install/setup.bash..."
# shellcheck disable=SC1091
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

echo "[run_multi_sensor_sim] Launching ground_truth_sensor_sim three_sensor_sim.launch.py"
ros2 launch ground_truth_sensor_sim three_sensor_sim.launch.py "$@"
