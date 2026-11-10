#!/usr/bin/env bash
set -e
BAI7_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
exec ros2 launch "$BAI7_DIR/launch/simulation.launch.py"
