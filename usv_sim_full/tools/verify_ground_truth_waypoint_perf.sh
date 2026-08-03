#!/usr/bin/env bash
# Ground truth waypoint 模式 open water 性能/稳定性门禁。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
elif [ -f "${HOME}/usv_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/usv_ws/install/setup.bash"
else
  echo "未找到 install/setup.bash，请先 colcon build 并 source" >&2
  exit 2
fi

PKG_SHARE="$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full"
CONFIG="${PKG_SHARE}/config/ground_truth_test.yaml"
MONITOR="${PKG_SHARE}/tools/monitor_ground_truth_counts.py"
LAUNCH_LOG="$(mktemp /tmp/gt_waypoint_launch_XXXX.log)"
MONITOR_LOG="$(mktemp /tmp/gt_waypoint_monitor_XXXX.log)"
HZ_LOG="$(mktemp /tmp/gt_waypoint_hz_XXXX.log)"

EXPECTED=3
MIN_HZ=45.0
WAIT_SPAWN_SEC=20
MONITOR_DURATION=120
MONITOR_WAIT=45

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "==> 启动 ground_truth_test (open water waypoint) ..."
ros2 launch usv_sim_full ground_truth_test.launch.py \
  config_path:="${CONFIG}" \
  gz_headless:=true \
  use_rviz:=false \
  verbose_launch:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "==> 等待 spawn_delay + Gazebo 就绪 (${WAIT_SPAWN_SEC}s) ..."
sleep "${WAIT_SPAWN_SEC}"

WORLD="$(python3 - <<'PY'
import yaml
from ament_index_python.packages import get_package_share_directory
import os
cfg = os.path.join(get_package_share_directory('usv_sim_full'), 'config', 'ground_truth_test.yaml')
with open(cfg, 'r', encoding='utf-8') as f:
    print(yaml.safe_load(f)['environment']['world_name'])
PY
)"

echo "==> 监测 track/gz 数量与航路 (${MONITOR_DURATION}s) ..."
if ! python3 "${MONITOR}" \
  --expected "${EXPECTED}" \
  --world "${WORLD}" \
  --duration-sec "${MONITOR_DURATION}" \
  --interval-sec 5 \
  --wait-topic-sec "${MONITOR_WAIT}" \
  --config "${CONFIG}" \
  --check-route \
  --route-tolerance-m 2.0 \
  --gz-violation-ratio-max 1.0 \
  | tee "${MONITOR_LOG}"; then
  echo "FAIL: monitor 未通过，见 ${MONITOR_LOG}" >&2
  echo "--- launch log tail ---" >&2
  tail -n 40 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

echo "==> 检查 /sim/ground_truth 频率 (>= ${MIN_HZ} Hz) ..."
if ! timeout 12 ros2 topic hz /sim/ground_truth 2>&1 | tee "${HZ_LOG}"; then
  echo "WARN: ros2 topic hz 超时，尝试解析已有输出" >&2
fi

HZ_AVG="$(grep -oP 'average rate: \K[0-9.]+' "${HZ_LOG}" | tail -n 1 || true)"
if [[ -z "${HZ_AVG}" ]]; then
  echo "FAIL: 未能解析 /sim/ground_truth 平均频率" >&2
  exit 1
fi

python3 - <<PY
hz = float("${HZ_AVG}")
min_hz = float("${MIN_HZ}")
print(f"平均频率: {hz:.2f} Hz (阈值 >= {min_hz})")
if hz < min_hz:
    raise SystemExit(1)
PY

echo "PASS: open water waypoint 性能门禁通过 (tracks=${EXPECTED}, hz=${HZ_AVG})"
