#!/usr/bin/env bash
# 干净环境直线航路轨迹测试：单目标、无 ping-pong、open water。
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "未找到 install/setup.bash，请先 colcon build 并 source" >&2
  exit 2
fi

PKG_SHARE="$(ros2 pkg prefix usv_sim_full)/share/usv_sim_full"
CONFIG="${PKG_SHARE}/config/ground_truth_straight_test.yaml"
ANALYZE="${PKG_SHARE}/tools/analyze_entity_trajectory.py"
LAUNCH_LOG="$(mktemp /tmp/gt_straight_launch_XXXX.log)"

WAIT_SPAWN_SEC=18
ANALYZE_DURATION=75

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "==> 启动直线航路隔离测试 (ground_truth_straight_test.yaml) ..."
ros2 launch usv_sim_full ground_truth_test.launch.py \
  config_path:="${CONFIG}" \
  gz_headless:=true \
  use_rviz:=false \
  verbose_launch:=false \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

echo "==> 等待 spawn (${WAIT_SPAWN_SEC}s) ..."
sleep "${WAIT_SPAWN_SEC}"

echo "==> 采样轨迹并检测回退 (${ANALYZE_DURATION}s) ..."
if ! python3 "${ANALYZE}" \
  --config "${CONFIG}" \
  --mode straight \
  --duration-sec "${ANALYZE_DURATION}" \
  --backward-speed-threshold 0.3 \
  --jump-threshold-m 8.0 \
  --min-samples 80; then
  echo "FAIL: 轨迹分析未通过，launch 日志尾部：" >&2
  tail -n 50 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

echo "PASS: 直线航路轨迹测试通过"
