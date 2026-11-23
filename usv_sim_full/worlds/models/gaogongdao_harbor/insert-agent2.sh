#!/usr/bin/env bash
set -euo pipefail
exec docker exec -e ROS_DOMAIN_ID=42 -e GZ_PARTITION=usv_ws_test_agent2 \
  usv_ws_test_agent2-dev-1 gz service -s /world/ccs_open_water_crop/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "/workspace/src/usv_simulation/usv_sim_full/worlds/models/gaogongdao_local_harbor_v1/model.sdf", name: "gaogongdao_local_harbor_v1", allow_renaming: false, pose: {position: {x: -170 y: 270 z: 0}}'
