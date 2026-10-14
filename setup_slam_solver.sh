#!/usr/bin/env bash
# Official SLAM Toolbox solver; only the worker count is adapted locally.
set -euo pipefail
btth4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
btth4_solver_revision=8293d21fe5c816d0405e0cc1f3eb46c70659c267
btth4_solver_source="$btth4_root/.deps/slam_solver_source"
btth4_solver_runtime="$btth4_root/.deps/slam_runtime"
btth4_solver_stamp="$btth4_solver_revision-workers-v2"
if [[ -f "$btth4_solver_runtime/lib/libceres_solver_plugin.so" &&
      -f "$btth4_solver_runtime/verified_revision" &&
      $(cat "$btth4_solver_runtime/verified_revision") == "$btth4_solver_stamp" ]]; then
  echo 'Workspace-local SLAM solver already built.'
  exit 0
fi
mkdir -p "$btth4_solver_source/solvers"
for btth4_file in solvers/ceres_solver.cpp solvers/ceres_solver.hpp solvers/ceres_utils.h LICENSE; do
  curl --fail --silent --show-error --location --retry 3 \
    "https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/$btth4_solver_revision/$btth4_file" \
    --output "$btth4_solver_source/$btth4_file"
done
python3 - "$btth4_solver_source/solvers/ceres_solver.cpp" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
old = 'options_.num_threads = 50;'
if s.count(old) != 1:
    raise SystemExit('Unexpected upstream solver; do not apply the worker patch.')
s = s.replace('#include <unordered_map>', '#include <algorithm>\n#include <thread>\n#include <unordered_map>', 1)
s = s.replace(old, 'options_.num_threads = std::max(1u, std::min(4u, std::thread::hardware_concurrency()));', 1)
p.write_text(s)
PY
set +u
source /opt/ros/humble/setup.bash
source "$btth4_root/.deps/install/local_setup.bash"
set -u
cmake -S "$btth4_root/runtime/slam_solver" -B "$btth4_root/.deps/build/slam_solver" \
  -DSOLVER_SOURCE="$btth4_solver_source" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$btth4_solver_runtime"
cmake --build "$btth4_root/.deps/build/slam_solver" --parallel 2
cmake --install "$btth4_root/.deps/build/slam_solver"
# Shadow only the solver library; official launch files and executables are reused.
mkdir -p "$btth4_solver_runtime/share/ament_index/resource_index/packages" \
  "$btth4_solver_runtime/share/ament_index/resource_index/slam_toolbox__pluginlib__plugin"
touch "$btth4_solver_runtime/share/ament_index/resource_index/packages/slam_toolbox"
cp /opt/ros/humble/share/ament_index/resource_index/slam_toolbox__pluginlib__plugin/slam_toolbox \
  "$btth4_solver_runtime/share/ament_index/resource_index/slam_toolbox__pluginlib__plugin/slam_toolbox"
ln -sfn /opt/ros/humble/share/slam_toolbox "$btth4_solver_runtime/share/slam_toolbox"
ln -sfn /opt/ros/humble/lib/slam_toolbox "$btth4_solver_runtime/lib/slam_toolbox"
printf '%s\n' "$btth4_solver_stamp" > "$btth4_solver_runtime/verified_revision"
