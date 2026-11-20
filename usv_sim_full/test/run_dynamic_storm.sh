#!/usr/bin/env bash
# Run the current dynamic-ship/storm regression set from any working directory.
set -euo pipefail

task_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_workspace="$(cd -- "$task_test_dir/../../../.." && pwd)"

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH="$task_test_dir/..:${PYTHONPATH:-}" \
python3 -m pytest -q \
  "$task_test_dir/test_dynamic_ship_placement.py" \
  "$task_test_dir/test_dynamic_ship_deletion.py" \
  "$task_test_dir/test_virtual_dynamic_ship.py" \
  "$task_test_dir/test_pose_feedback.py" \
  "$task_test_dir/test_storm_field_manager_node.py"

ctest --test-dir "$task_workspace/build/env_panel" --output-on-failure \
  -R '^test_(target_list|panel_interaction)$' --no-tests=error
