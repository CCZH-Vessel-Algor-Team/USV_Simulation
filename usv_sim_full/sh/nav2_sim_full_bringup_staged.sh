#!/usr/bin/env bash
# 目的：不替代 launch 逻辑，仅包装调用 nav2_sim_full_bringup.launch.py，
#       将详细输出写入日志，并在终端按阶段显示「仿真 / RViz / 场景与障碍物 / Nav2」就绪状态。
# 包：usv_sim_full
# 验证：bash sh/nav2_sim_full_bringup_staged.sh
#       或（安装后）ros2 pkg prefix usv_sim_full → share/.../sh/ 下执行

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 工作区根：从本脚本向上找到含 install/setup.bash 或 src 的目录
find_ws_root() {
  local d="$SCRIPT_DIR"
  while [[ "$d" != "/" ]]; do
    if [[ -f "$d/install/setup.bash" ]] || [[ -d "$d/src/usv_simulation/usv_sim_full" ]]; then
      echo "$d"
      return 0
    fi
    d="$(dirname "$d")"
  done
  echo "$SCRIPT_DIR/../../.." # 尽力回退：sh -> usv_sim_full -> usv_simulation -> 工作区根
}

WS_ROOT="$(find_ws_root)"
REDIRECT_LOG=1
POLL_INTERVAL="${POLL_INTERVAL:-1.5}"
LAUNCH_PKG="usv_sim_full"
LAUNCH_FILE="nav2_sim_full_bringup.launch.py"
LAUNCH_ARGS=()

usage() {
  cat <<EOF
nav2_sim_full_bringup_staged.sh — 包装 ros2 launch usv_sim_full nav2_sim_full_bringup.launch.py，终端分阶段就绪提示。

用法:
  $0 [本脚本选项] -- [传给 ros2 launch 的参数]

本脚本选项（须写在单独 -- 之前）:
  --no-log-redirect    不把 launch 输出重定向到文件（终端会与阶段行交错）
  --poll-interval SEC  轮询周期，默认 ${POLL_INTERVAL}（也可用环境变量 POLL_INTERVAL）

示例:
  $0
  $0 -- config_path:=/path/to/full_config.yaml nav2_start_delay:=30.0
  POLL_INTERVAL=1.0 $0 -- verbose_launch:=true

说明:
  - 不包含 colcon 编译；请先 source install/setup.bash。
  - 若未设置 ROS_DISTRO，将尝试 source \"$WS_ROOT/install/setup.bash\"。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-log-redirect)
      REDIRECT_LOG=0
      shift
      ;;
    --poll-interval)
      POLL_INTERVAL="$2"
      shift 2
      ;;
    --)
      shift
      LAUNCH_ARGS+=("$@")
      break
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      shift
      ;;
  esac
done

# 解析 config_path / nav2_start_delay / nav2_namespace（与 launch 默认值对齐）
DEFAULT_CONFIG_SHARE=""
if command -v ros2 &>/dev/null; then
  DEFAULT_CONFIG_SHARE="$(ros2 pkg prefix "${LAUNCH_PKG}" 2>/dev/null)/share/${LAUNCH_PKG}/config/full_config.yaml" || true
fi
if [[ ! -f "${DEFAULT_CONFIG_SHARE:-}" ]]; then
  DEFAULT_CONFIG_SHARE="$(cd "$SCRIPT_DIR/.." && pwd)/config/full_config.yaml"
fi

CONFIG_PATH="${DEFAULT_CONFIG_SHARE}"
NAV2_DELAY="25.0"
NAV2_NS="auto"
for a in "${LAUNCH_ARGS[@]}"; do
  case "$a" in
    config_path:=*) CONFIG_PATH="${a#config_path:=}" ;;
    nav2_start_delay:=*) NAV2_DELAY="${a#nav2_start_delay:=}" ;;
    nav2_namespace:=*) NAV2_NS="${a#nav2_namespace:=}" ;;
  esac
done

if [[ ! -f "$CONFIG_PATH" ]]; then
  echo "[staged] 警告: 找不到配置文件: $CONFIG_PATH（将仍启动 launch，但阶段判断可能不准）" >&2
fi

resolve_primary_ns() {
  python3 - "$CONFIG_PATH" "$NAV2_NS" <<'PY'
import sys
from pathlib import Path

cfg_path = Path(sys.argv[1])
requested = (sys.argv[2] or "auto").strip()

def primary_from_yaml(p: Path) -> str:
    try:
        import yaml  # type: ignore
        d = yaml.safe_load(p.read_text()) or {}
    except Exception:
        return "usv_1"
    keys = sorted(
        [k for k in d if k.startswith("robot_") and len(k) > 6 and k[6:].isdigit()],
        key=lambda x: int(x.split("_")[1]),
    )
    if keys:
        n = d[keys[0]].get("name")
        return str(n).strip() if n else "usv_1"
    rb = d.get("robot") or {}
    n = rb.get("name")
    return str(n).strip() if n else "usv_1"

if requested and requested.lower() not in ("auto", ""):
    print(requested)
else:
    print(primary_from_yaml(cfg_path) if cfg_path.is_file() else "usv_1")
PY
}

RESOLVED_NS="$(resolve_primary_ns)"

launch_rviz_enabled() {
  python3 - "$CONFIG_PATH" <<'PY'
import sys
from pathlib import Path
try:
    import yaml  # type: ignore
    p = Path(sys.argv[1])
    if not p.is_file():
        print("true")
        raise SystemExit
    d = yaml.safe_load(p.read_text()) or {}
    v = d.get("visualization") or {}
    print("true" if v.get("launch_rviz", True) else "false")
except Exception:
    print("true")
PY
}

RVIZ_CFG="$(launch_rviz_enabled)"

# Source ROS
if [[ -z "${ROS_DISTRO:-}" && -f "$WS_ROOT/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/setup.bash"
fi

if ! command -v ros2 &>/dev/null; then
  echo "[staged] 错误: 未找到 ros2。请先: source <工作区>/install/setup.bash" >&2
  exit 1
fi

LOG_FILE=""
if [[ "$REDIRECT_LOG" == 1 ]]; then
  LOG_FILE="$(mktemp -t usv_nav2_bringup_XXXXXX.log)"
fi

# ---- 终端阶段显示（stderr，避免与文件重定向混淆）----
C_RESET=$'\033[0m'
C_DIM=$'\033[2m'
C_OK=$'\033[32m'
C_WAIT=$'\033[33m'
C_SKIP=$'\033[90m'
C_ERR=$'\033[31m'

S_SIM=0
S_RVIZ=0
S_OBS=0
S_NAV2=0
PANEL_HASH=""

status_line() {
  local code="$1"
  local text="$2"
  case "$code" in
    ok)   echo -e "  ${C_OK}✓${C_RESET}  $text" ;;
    wait) echo -e "  ${C_WAIT}…${C_RESET}  $text" ;;
    skip) echo -e "  ${C_SKIP}○${C_RESET}  $text" ;;
    err)  echo -e "  ${C_ERR}✗${C_RESET}  $text" ;;
    *)    echo "  ?  $text" ;;
  esac
}

draw_panel() {
  # 保存光标、上移到面板顶、清屏块（固定 9 行）
  local sim_st rviz_st obs_st nav2_st
  case "$S_SIM" in 1) sim_st="ok" ;; 2) sim_st="skip" ;; *) sim_st="wait" ;; esac
  case "$S_RVIZ" in 1) rviz_st="ok" ;; 2) rviz_st="skip" ;; *) rviz_st="wait" ;; esac
  case "$S_OBS" in 1) obs_st="ok" ;; 2) obs_st="skip" ;; *) obs_st="wait" ;; esac
  case "$S_NAV2" in 1) nav2_st="ok" ;; 2) nav2_st="skip" ;; *) nav2_st="wait" ;; esac

  echo "" >&2
  echo -e "${C_DIM}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}" >&2
  echo -e " ${C_OK}USV${C_RESET}  nav2_sim_full_bringup  ${C_DIM}│${C_RESET}  主船命名空间: ${C_OK}${RESOLVED_NS}${C_RESET}" >&2
  echo -e "${C_DIM}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}" >&2
  if [[ "$S_SIM" -eq 1 ]]; then
    status_line ok "Gazebo / 仿真时钟 — 已就绪"
  elif [[ "$S_SIM" -eq 2 ]]; then
    status_line skip "Gazebo / 仿真时钟 — 未检测到（请查看日志）"
  else
    status_line wait "Gazebo / 仿真时钟 — 等待 /clock 与仿真进程…"
  fi
  if [[ "$RVIZ_CFG" != "true" ]]; then
    status_line skip "RViz — 配置 visualization.launch_rviz=false，跳过"
  elif [[ "$S_RVIZ" -eq 1 ]]; then
    status_line ok "RViz — 已启动"
  else
    status_line wait "RViz — 等待 rviz2 进程…"
  fi
  if [[ "$S_OBS" -eq 1 ]]; then
    status_line ok "场景与障碍物 — scenario_manager（及 obstacle_spawner 若启用）已就绪"
  else
    status_line wait "场景与障碍物 — 等待 scenario_manager_node / obstacle_spawner…"
  fi
  if [[ "$S_NAV2" -eq 1 ]]; then
    status_line ok "Nav2 — controller_server 已出现（导航栈已拉起）"
  else
    status_line wait "Nav2 — 等待延时后 nav2_thruster_bringup（约 ${NAV2_DELAY}s）…"
  fi
  echo -e "${C_DIM}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${C_RESET}" >&2
  if [[ -n "$LOG_FILE" ]]; then
    echo -e " ${C_DIM}详细日志:${C_RESET} $LOG_FILE" >&2
  fi
}

nodes_snapshot() {
  timeout 6 ros2 node list 2>/dev/null || true
}

have_clock() {
  ros2 topic list 2>/dev/null | grep -qE '^/clock$'
}

have_gz() {
  pgrep -af 'gz sim' >/dev/null 2>&1 || pgrep -af 'ruby.*gz sim' >/dev/null 2>&1
}

have_rviz() {
  pgrep -af '[r]viz2' >/dev/null 2>&1
}

have_nav2() {
  local ns="$1"
  nodes_snapshot | grep -qE "/${ns}/controller_server" 2>/dev/null && return 0
  nodes_snapshot | grep -qE 'controller_server' 2>/dev/null
}

have_scenario() {
  nodes_snapshot | grep -qE 'scenario_manager_node' 2>/dev/null
}

have_obstacle_spawner() {
  nodes_snapshot | grep -qE 'obstacle_spawner' 2>/dev/null
}

panel_refresh() {
  local h="${S_SIM}${S_RVIZ}${S_OBS}${S_NAV2}"
  if [[ "$h" != "$PANEL_HASH" ]]; then
    PANEL_HASH="$h"
    clear >&2 2>/dev/null || true
    draw_panel
  fi
}

monitor_loop() {
  local start elapsed scen_t0=""
  start="$(date +%s)"
  if [[ "$RVIZ_CFG" != "true" ]]; then
    S_RVIZ=2
  fi
  while true; do
    if [[ -n "${LAUNCH_PID:-}" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo -e "\n${C_ERR}[staged] ros2 launch 进程已结束，停止阶段监控。${C_RESET}" >&2
      break
    fi

    local nodes
    nodes="$(nodes_snapshot)"

    if [[ "$S_SIM" -eq 0 ]]; then
      if have_clock || have_gz; then
        S_SIM=1
      fi
    fi

    if [[ "$RVIZ_CFG" == "true" && "$S_RVIZ" -eq 0 && "$S_SIM" -eq 1 ]]; then
      if have_rviz; then
        S_RVIZ=1
      fi
    fi

    if [[ "$S_OBS" -eq 0 ]]; then
      if echo "$nodes" | grep -qE 'scenario_manager_node'; then
        [[ -z "$scen_t0" ]] && scen_t0="$(date +%s)"
        if echo "$nodes" | grep -qE 'obstacle_spawner'; then
          S_OBS=1
        else
          local sc_elapsed
          sc_elapsed="$(($(date +%s) - scen_t0))"
          if [[ "$S_SIM" -eq 1 && "$sc_elapsed" -ge 18 ]]; then
            S_OBS=1
          fi
        fi
      fi
    fi

    if [[ "$S_NAV2" -eq 0 ]]; then
      if have_nav2 "$RESOLVED_NS"; then
        S_NAV2=1
      fi
    fi

    panel_refresh

    if [[ "$S_SIM" -eq 1 && "$S_OBS" -eq 1 && "$S_NAV2" -eq 1 ]]; then
      if [[ "$RVIZ_CFG" != "true" || "$S_RVIZ" -ge 1 ]]; then
        echo -e "\n${C_OK}[staged] 全部关键阶段已就绪（仿真 / 场景 / Nav2；RViz 按配置）。${C_RESET}" >&2
        break
      fi
    fi

    elapsed="$(($(date +%s) - start))"
    if [[ "$S_SIM" -eq 0 && "$elapsed" -gt 600 ]]; then
      echo -e "\n${C_ERR}[staged] 超过 10 分钟仍未检测到 /clock 或 gz sim；请检查日志。${C_RESET}" >&2
      break
    fi

    sleep "$POLL_INTERVAL"
  done
}

LAUNCH_PID=""
MON_PID=""
cleanup() {
  if [[ -n "${MON_PID:-}" ]]; then
    kill "$MON_PID" 2>/dev/null || true
    wait "$MON_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo -e "\n${C_DIM}[staged] 收到中断，结束 ros2 launch (pid=$LAUNCH_PID)…${C_RESET}" >&2
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM

echo -e "[staged] 启动: ros2 launch ${LAUNCH_PKG} ${LAUNCH_FILE} ${LAUNCH_ARGS[*]}" >&2
echo -e "[staged] Nav2 延时参数 nav2_start_delay=${NAV2_DELAY}（与 launch 一致）" >&2

if [[ "$REDIRECT_LOG" == 1 ]]; then
  ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" >"$LOG_FILE" 2>&1 &
  LAUNCH_PID=$!
  echo -e "[staged] Launch pid=$LAUNCH_PID" >&2
else
  # 仍后台跑以便监控；输出到终端会乱，不推荐
  ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" "${LAUNCH_ARGS[@]}" &
  LAUNCH_PID=$!
fi

# 等 ROS 守护进程就绪
sleep 1
PANEL_HASH=""
draw_panel
monitor_loop &
MON_PID=$!

wait "$LAUNCH_PID"
EXIT_CODE=$?
kill "$MON_PID" 2>/dev/null || true
wait "$MON_PID" 2>/dev/null || true

trap - INT TERM
echo -e "\n[staged] ros2 launch 已退出，退出码: $EXIT_CODE" >&2
if [[ -n "$LOG_FILE" ]]; then
  echo -e "[staged] 完整日志仍保留于: $LOG_FILE" >&2
fi
exit "$EXIT_CODE"
