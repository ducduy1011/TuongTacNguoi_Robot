#!/usr/bin/env bash
# Always use the same ROS domain and middleware as the BTTH04 simulator.
set -e
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
# Reset after each simulator restart; retain bumper/cliff reflexes.
if ! timeout 10 ros2 param set /motion_control safety_override backup_only --no-daemon --spin-time 3; then
  echo 'Motion control is not ready. Start the simulator and wait before opening WASD.' >&2
  exit 1
fi
# Vietnamese input methods compose W/A/S/D instead of passing each key through.
if command -v ibus >/dev/null 2>&1; then
  btth4_keyboard_engine="$(ibus engine 2>/dev/null || true)"
  if [[ -n "$btth4_keyboard_engine" && "$btth4_keyboard_engine" != xkb:* ]]; then
    if ibus engine xkb:us::eng >/dev/null 2>&1; then
      echo 'WASD: English keyboard enabled; previous input method is restored on exit.'
      trap 'ibus engine "$btth4_keyboard_engine" >/dev/null 2>&1 || true' EXIT
    fi
  fi
fi
ros2 run turtlebot4_btth4 teleop_wasd --ros-args \
  -p speed:=0.5 -p turn:=0.5 -p use_sim_time:=true
