#!/usr/bin/env bash
# 开放水域 ping-pong 折返 + 非预期倒退检测。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "未找到 install/setup.bash" >&2
  exit 2
fi

PKG_SHARE="$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full"
CONFIG="${PKG_SHARE}/config/ground_truth_pingpong_open_water.yaml"
ANALYZE="${PKG_SHARE}/tools/analyze_entity_trajectory.py"
LAUNCH_LOG="$(mktemp /tmp/gt_pingpong_launch_XXXX.log)"

WAIT_SPAWN_SEC=18
# 单程 160m @ 3.5m/s ≈ 46s，往返 + 掉头需 >100s
ANALYZE_DURATION=130

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "==> 开放水域 ping-pong 折返测试 ..."
ros2 launch usv_sim_full ground_truth_test.launch.py \
  config_path:="${CONFIG}" \
  gz_headless:=true \
  use_rviz:=false \
  verbose_launch:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "==> 等待 spawn (${WAIT_SPAWN_SEC}s) ..."
sleep "${WAIT_SPAWN_SEC}"

echo "==> 采样轨迹：折返 + 非预期倒退检测 (${ANALYZE_DURATION}s) ..."
if ! python3 "${ANALYZE}" \
  --config "${CONFIG}" \
  --mode pingpong \
  --duration-sec "${ANALYZE_DURATION}" \
  --backward-speed-threshold 0.5 \
  --turn-zone-m 25.0 \
  --endpoint-reach-ratio 0.7 \
  --min-samples 300; then
  echo "FAIL: ping-pong 轨迹分析未通过" >&2
  tail -n 60 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

echo "PASS: 开放水域 ping-pong 折返测试通过"
