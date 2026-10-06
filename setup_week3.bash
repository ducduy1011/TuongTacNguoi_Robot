# Source this file in each terminal. Never source the Week 2 workspace.
_week3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$_week3_root/install/local_setup.bash" ]]; then
  source "$_week3_root/install/local_setup.bash"
fi
export ROS_DOMAIN_ID=73
export IGN_PARTITION=ur3_week3
export ROS_LOG_DIR="$_week3_root/log/ros"
export ROS_HOME="$_week3_root/log/ros_home"
export IGN_HOMEDIR="$_week3_root/log/ign"
export GZ_HOMEDIR="$IGN_HOMEDIR"
export TMPDIR="$_week3_root/tmp"
export XDG_CACHE_HOME="$TMPDIR/cache"
export XDG_CONFIG_HOME="$TMPDIR/config"
export XDG_DATA_HOME="$TMPDIR/data"
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$IGN_HOMEDIR" "$TMPDIR" "$XDG_CACHE_HOME" "$XDG_CONFIG_HOME" "$XDG_DATA_HOME"
unset _week3_root
