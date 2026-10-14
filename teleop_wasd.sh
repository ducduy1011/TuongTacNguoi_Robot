#!/usr/bin/env bash
# Always use the same ROS domain and middleware as the BTTH04 simulator.
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec ros2 run turtlebot4_btth4 teleop_wasd --ros-args \
  -p speed:=0.5 -p turn:=0.5 -p use_sim_time:=true
