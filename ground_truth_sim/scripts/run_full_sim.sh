#!/usr/bin/env bash
set -euo pipefail

# Automatically clean, rebuild, source, and launch the full simulation stack.
# Usage: ./scripts/run_full_sim.sh

SCRIPT_DIR="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR%/*}"

strip_ws_paths() {
  local var_name=$1
  local current_value="${!var_name-}"
  [[ -z "${current_value}" ]] && return 0

  local IFS=':'
  local -a paths=()
  local -a filtered=()
  read -ra paths <<< "${current_value}"
  for entry in "${paths[@]}"; do
    [[ -z "${entry}" ]] && continue
    if [[ "${entry}" == ${WS_DIR}* ]]; then
      continue
    fi
    filtered+=("${entry}")
  done

  local joined="${filtered[*]:-}"
  joined=${joined// /:}
  export "${var_name}=${joined}"
}

cd "${WS_DIR}"

if [[ -z "${ROS_DISTRO:-}" ]];
then
  echo "[WARN] ROS_DISTRO is not set. Please source your ROS 2 environment (e.g., /opt/ros/<distro>/setup.bash) before running this script." >&2
fi

strip_ws_paths AMENT_PREFIX_PATH
strip_ws_paths COLCON_PREFIX_PATH
strip_ws_paths CMAKE_PREFIX_PATH

echo "[INFO] Cleaning build/install/log directories..."
rm -rf build install log

echo "[INFO] Running colcon build..."
colcon build

echo "[INFO] Sourcing install/setup.bash..."
# shellcheck disable=SC1090
set +u
source "${WS_DIR}/install/setup.bash"
set -u

echo "[INFO] Launching ground_truth_sim with vision + mmWave sensor sim..."
ros2 launch ground_truth_sim ground_truth_sim.launch.py \
  start_vision_node:=true \
  start_mmwave_node:=true
