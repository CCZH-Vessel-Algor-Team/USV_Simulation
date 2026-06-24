#!/usr/bin/env bash
set -euo pipefail

# Quick start helper for USV_Simulation
# From the repository root this script will:
# 1. Build core packages with colcon
# 2. Source the workspace
# 3. Export common GZ/GAZEBO model paths
# 4. Launch the main ROS2 launch and write a PID file + log

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"

echo "Repository root: $ROOT"

# Stop any previous launch from this helper (prevents duplicate bridges/topics)
PIDFILE="$ROOT/.usv_sim_pid"
if [[ -f "$PIDFILE" ]]; then
	OLD_PID="$(cat "$PIDFILE" || true)"
	if [[ -n "${OLD_PID}" ]] && kill -0 "$OLD_PID" 2>/dev/null; then
		echo "==> Stopping previous simulation (PID=$OLD_PID)"
		kill "$OLD_PID" || true
		sleep 2
	fi
	rm -f "$PIDFILE"
fi

if pgrep -f "ros2 launch usv_sim_full main.launch.py" >/dev/null 2>&1; then
	echo "==> Stopping existing usv_sim_full launch processes"
	pkill -f "ros2 launch usv_sim_full main.launch.py" || true
	sleep 2
fi

echo "==> Building core packages (this may take a while)..."
colcon build 

echo "==> Sourcing install/setup.bash"
# Some setup scripts assume unset variables (e.g. COLCON_TRACE). Our script enabled
# nounset (set -u) at the top which would make sourcing fail if the setup script
# references an unset variable. Temporarily disable nounset while sourcing.
set +u
# shellcheck source=/dev/null
source "$ROOT/install/setup.bash"
set -u

echo "==> Setting simulation resource paths"
# Make sure we include the 'share' directory so model://usv_sim_full references work
export GZ_SIM_RESOURCE_PATH="$ROOT/install/usv_sim_full/share:$ROOT/install/usv_sim_full/share/usv_sim_full/description:$ROOT/install/vrx_gz/share/vrx_gz/models:$ROOT/install/vrx_gazebo/share/vrx_gazebo/models:$ROOT/install/wamv_description/share/wamv_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export GAZEBO_MODEL_PATH="$ROOT/install/wamv_description/share/wamv_description/models:$ROOT/install/vrx_gazebo/share/vrx_gazebo/models:$ROOT/install/usv_sim_full/share/usv_sim_full/description:${GAZEBO_MODEL_PATH:-}"

echo "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"

LOG="$ROOT/usv_sim_start.log"
PIDFILE="$ROOT/.usv_sim_pid"

echo "==> Launching simulation (output -> $LOG)"
ros2 launch usv_sim_full main.launch.py config_path:='./src/usv_sim_full/config/full_config.yaml' &> "$LOG" &
echo $! > "$PIDFILE"
echo "Launched. PID=$(cat $PIDFILE)"

echo "Waiting briefly for processes to start..."
sleep 3

echo "--- Log head ($LOG) ---"
sed -n '1,200p' "$LOG" || true
echo "--- end ---"

echo "To stop the simulation:"
echo "  kill \\$(cat $PIDFILE) && rm -f $PIDFILE"

exit 0
