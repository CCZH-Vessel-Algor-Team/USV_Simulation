#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${PKG_DIR}/../../.." && pwd)"
WORLD_FILE="${PKG_DIR}/worlds/ocean.world"
LOG_FILE="/tmp/ocean_world_check.log"
TIMEOUT_SEC="${TIMEOUT_SEC:-20}"

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "[ERROR] World file not found: ${WORLD_FILE}"
  exit 1
fi

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "${WS_DIR}/install/setup.bash"
  set -u
else
  echo "[WARN] install/setup.bash not found under ${WS_DIR}, continuing with current environment"
fi

prepend_path() {
  local var_name="$1"
  local new_path="$2"
  local old_value="${!var_name:-}"
  if [[ -n "${old_value}" ]]; then
    printf -v "${var_name}" "%s:%s" "${new_path}" "${old_value}"
  else
    printf -v "${var_name}" "%s" "${new_path}"
  fi
}

prepend_path GZ_SIM_RESOURCE_PATH "${PKG_DIR}/worlds"
prepend_path GZ_SIM_RESOURCE_PATH "${PKG_DIR}/worlds/models"
prepend_path GZ_SIM_RESOURCE_PATH "${PKG_DIR}/description"
prepend_path GZ_SIM_RESOURCE_PATH "${PKG_DIR}/description/models"

prepend_path GAZEBO_MODEL_PATH "${PKG_DIR}/worlds"
prepend_path GAZEBO_MODEL_PATH "${PKG_DIR}/worlds/models"
prepend_path GAZEBO_MODEL_PATH "${PKG_DIR}/description"
prepend_path GAZEBO_MODEL_PATH "${PKG_DIR}/description/models"

export GZ_SIM_RESOURCE_PATH
export GAZEBO_MODEL_PATH

echo "[INFO] Launching Gazebo with ${WORLD_FILE}"
echo "[INFO] Log: ${LOG_FILE}"

set +e
timeout "${TIMEOUT_SEC}" ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-r ${WORLD_FILE}" >"${LOG_FILE}" 2>&1
launch_rc=$?
set -e

if [[ ${launch_rc} -ne 0 && ${launch_rc} -ne 124 ]]; then
  echo "[ERROR] Launch command exited with unexpected code: ${launch_rc}"
  tail -n 80 "${LOG_FILE}" || true
  exit ${launch_rc}
fi

grep -iE "error|failed|unable to find|could not resolve|exception|traceback" "${LOG_FILE}" \
  | grep -viE "libEGL warning: egl: failed to create dri2 screen" \
  > /tmp/ocean_world_errors.log || true

if [[ -s /tmp/ocean_world_errors.log ]]; then
  echo "[WARN] Potential issues found while loading ocean.world:"
  cat /tmp/ocean_world_errors.log
  echo "[WARN] Full log: ${LOG_FILE}"
  exit 2
fi

echo "[OK] ocean.world loaded without detected error patterns in ${TIMEOUT_SEC}s window."
