#!/usr/bin/env bash
set -euo pipefail
btth4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source "$btth4_root/env.sh"
set -u
mkdir -p "$btth4_root/src/turtlebot4_btth4/maps"
ros2 run nav2_map_server map_saver_cli \
  -f "$btth4_root/src/turtlebot4_btth4/maps/map" \
  --free 0.196 --occ 0.65 --fmt pgm --mode trinary \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=20.0 -p free_thresh_default:=0.196
echo "Saved map.yaml and map.pgm in $btth4_root/src/turtlebot4_btth4/maps"
