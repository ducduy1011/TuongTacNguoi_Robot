#!/usr/bin/env bash
# Workspace-local TF2, SLAM worker limit and simulator controller configuration.
set -euo pipefail
btth4_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
btth4_deps="$btth4_root/.deps"
btth4_revision=65c620c308920f558c7b5d3fb852941bd6d8fced
if [[ -f "$btth4_deps/install/tf2_ros/lib/libtf2_ros.so" &&
      -f "$btth4_deps/verified_revision" &&
      $(cat "$btth4_deps/verified_revision") == "$btth4_revision" ]]; then
  echo 'Workspace-local TF2 compatibility library already built.'
else
  mkdir -p "$btth4_deps/src"
  if [[ ! -d "$btth4_deps/src/geometry2/.git" ]]; then
    git clone --depth 1 --branch 0.25.22 \
      https://github.com/ros2/geometry2.git "$btth4_deps/src/geometry2"
  fi
  [[ $(git -C "$btth4_deps/src/geometry2" rev-parse HEAD) == "$btth4_revision" ]]
  # Only the official library is built; /opt/ros and other workspaces are untouched.
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  cd "$btth4_root"
  CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
    --base-paths "$btth4_deps/src/geometry2/tf2_ros" \
    --build-base "$btth4_deps/build" --install-base "$btth4_deps/install" \
    --packages-select tf2_ros --allow-overriding tf2_ros \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
  printf '%s\n' "$btth4_revision" > "$btth4_deps/verified_revision"
fi
bash "$btth4_root/setup_slam_solver.sh"
python3 - "$btth4_deps/control_runtime" <<'PY'
from pathlib import Path
import sys
import yaml
root = Path(sys.argv[1])
original = Path('/opt/ros/humble/share/irobot_create_control')
share = root / 'share' / 'irobot_create_control'
(share / 'config').mkdir(parents=True, exist_ok=True)
for p in original.iterdir():
    if p.name != 'config':
        target = share / p.name
        if not target.exists(): target.symlink_to(p, target_is_directory=p.is_dir())
for p in (original / 'config').iterdir():
    target = share / 'config' / p.name
    if p.name == 'control.yaml':
        if target.is_symlink(): target.unlink()
        data = yaml.safe_load(p.read_text())
        data['/**']['controller_manager']['ros__parameters']['update_rate'] = 100
        target.write_text(yaml.safe_dump(data, sort_keys=False))
    elif not target.exists(): target.symlink_to(p, target_is_directory=p.is_dir())
index = root / 'share/ament_index/resource_index/packages'
index.mkdir(parents=True, exist_ok=True)
(index / 'irobot_create_control').touch()
print('Official Create 3 controller configuration: 100 Hz, compatible with maze physics.')
PY
