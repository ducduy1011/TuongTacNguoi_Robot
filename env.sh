# Source this file in every terminal used for BTTH4.
_btth4_ws="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$_btth4_ws/.deps/install/local_setup.bash" ]]; then
  source "$_btth4_ws/.deps/install/local_setup.bash"
fi
if [[ -f "$_btth4_ws/install/local_setup.bash" ]]; then
  source "$_btth4_ws/install/local_setup.bash"
fi
export ROS_DOMAIN_ID=44
if [[ -d "$_btth4_ws/.deps/control_runtime/share" ]]; then
  export AMENT_PREFIX_PATH="$_btth4_ws/.deps/control_runtime:$AMENT_PREFIX_PATH"
fi
if [[ -f "$_btth4_ws/.deps/slam_runtime/lib/libceres_solver_plugin.so" ]]; then
  export AMENT_PREFIX_PATH="$_btth4_ws/.deps/slam_runtime:$AMENT_PREFIX_PATH"
  export LD_LIBRARY_PATH="$_btth4_ws/.deps/slam_runtime/lib:$LD_LIBRARY_PATH"
fi
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$_btth4_ws/src/turtlebot4_btth4/config/fastdds.xml"
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS
unset CYCLONEDDS_URI
export RMW_FASTRTPS_USE_QOS_FROM_XML=0
export IGN_PARTITION=turtlebot4_btth4
export IGN_IP=127.0.0.1
export GZ_IP=127.0.0.1
# NVIDIA avoids broken Ogre sensor/map rendering on this hybrid-GPU laptop.
if [[ -f /proc/driver/nvidia/version ]]; then
  export __NV_PRIME_RENDER_OFFLOAD=1
  export __GLX_VENDOR_LIBRARY_NAME=nvidia
fi
export ROS_LOG_DIR="$_btth4_ws/log/ros"
mkdir -p "$ROS_LOG_DIR"
unset _btth4_ws
