#!/usr/bin/env bash
# Gazebo 实体权威真值对齐门禁：发布的 /sim/ground_truth 与 gz pose 应一致（dXY<=0.5m）。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

cleanup_stale_sim() {
  pkill -f "ground_truth_test.launch.py" 2>/dev/null || true
  pkill -f "ground_truth_gazebo_entity_node" 2>/dev/null || true
  pkill -f "gz sim.*sydney_regatta_open_water" 2>/dev/null || true
  sleep 2
}
cleanup_stale_sim

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "未找到 install/setup.bash，请先 colcon build" >&2
  exit 2
fi

PKG_SHARE="$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full"
CONFIG="${PKG_SHARE}/config/ground_truth_test.yaml"
COMPARE="${PKG_SHARE}/tools/compare_gt_gz_entity_truth.py"
LAUNCH_LOG="$(mktemp /tmp/gt_entity_launch_XXXX.log)"

EXPECTED=3
WAIT_SPAWN_SEC=35
MONITOR_SAMPLES=5

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "==> 启动 ground_truth_test (entity 权威, open water) ..."
ros2 launch usv_sim_full ground_truth_test.launch.py \
  config_path:="${CONFIG}" \
  gz_headless:=true \
  use_rviz:=false \
  verbose_launch:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "==> 等待 spawn (${WAIT_SPAWN_SEC}s) ..."
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

echo "==> 检查 /sim/ground_truth 发布者数量 ..."
PUB_COUNT="$(ros2 topic info /sim/ground_truth -v 2>/dev/null | awk '/^Publisher count:/ {print $3; exit}')"
if [ "${PUB_COUNT}" = "1" ]; then
  echo "  Publisher count = 1 OK"
else
  echo "  期望 Publisher count=1，实际=${PUB_COUNT:-unknown}" >&2
  ros2 topic info /sim/ground_truth -v 2>/dev/null || true
  exit 1
fi

echo "==> 对比 truth vs Gazebo pose (world=${WORLD}) ..."
python3 "${COMPARE}" \
  --world "${WORLD}" \
  --prefix gt_ctrv_ \
  --samples "${MONITOR_SAMPLES}" \
  --interval-sec 2 \
  --max-dxy-m 0.5 \
  --wait-topic-sec 60

echo "==> verify_entity_truth_alignment: PASS"
