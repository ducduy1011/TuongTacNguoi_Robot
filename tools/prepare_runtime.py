#!/usr/bin/env python3
"""Prepare local compatibility libraries for ROS 2 Humble and Gazebo Fortress."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import urllib.request

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
DEPS = WORKSPACE / '.deps'
TF2_REVISION = '65c620c308920f558c7b5d3fb852941bd6d8fced'
SLAM_REVISION = '8293d21fe5c816d0405e0cc1f3eb46c70659c267'


def run(arguments, env=None):
    subprocess.run(arguments, cwd=WORKSPACE, env=env, check=True)


def prepared(library, marker, revision):
    return library.is_file() and marker.is_file() and marker.read_text().strip() == revision


def prepare_tf2():
    marker = DEPS / 'verified_revision'
    if prepared(DEPS / 'install/tf2_ros/lib/libtf2_ros.so', marker, TF2_REVISION):
        print('Local TF2 compatibility library is ready.', flush=True)
        return
    source = DEPS / 'src/geometry2'
    source.parent.mkdir(parents=True, exist_ok=True)
    if not (source / '.git').is_dir():
        run(['git', 'clone', '--depth', '1', '--branch', '0.25.22',
             'https://github.com/ros2/geometry2.git', str(source)])
    revision = subprocess.check_output(
        ['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
    if revision != TF2_REVISION:
        raise RuntimeError('The local TF2 source does not match the pinned revision.')
    env = os.environ.copy()
    env['CMAKE_BUILD_PARALLEL_LEVEL'] = '2'
    run(['colcon', 'build', '--base-paths', str(source / 'tf2_ros'),
         '--build-base', str(DEPS / 'build'), '--install-base', str(DEPS / 'install'),
         '--packages-select', 'tf2_ros', '--allow-overriding', 'tf2_ros',
         '--cmake-args', '-DBUILD_TESTING=OFF', '-DCMAKE_BUILD_TYPE=Release'], env)
    marker.write_text(TF2_REVISION + '\n')


def tf2_environment():
    env = os.environ.copy()
    prefix = DEPS / 'install/tf2_ros'
    for key, directory in [('AMENT_PREFIX_PATH', prefix), ('CMAKE_PREFIX_PATH', prefix),
                           ('LD_LIBRARY_PATH', prefix / 'lib')]:
        env[key] = str(directory) + (os.pathsep + env[key] if env.get(key) else '')
    return env


def prepare_solver():
    source = DEPS / 'slam_solver_source'
    runtime = DEPS / 'slam_runtime'
    marker = runtime / 'verified_revision'
    stamp = SLAM_REVISION + '-workers-v2'
    if prepared(runtime / 'lib/libceres_solver_plugin.so', marker, stamp):
        print('Local SLAM solver is ready.', flush=True)
        return
    (source / 'solvers').mkdir(parents=True, exist_ok=True)
    for name in ['solvers/ceres_solver.cpp', 'solvers/ceres_solver.hpp',
                 'solvers/ceres_utils.h', 'LICENSE']:
        url = f'https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/{SLAM_REVISION}/{name}'
        with urllib.request.urlopen(url, timeout=60) as response:
            (source / name).write_bytes(response.read())
    solver = source / 'solvers/ceres_solver.cpp'
    text = solver.read_text()
    old = 'options_.num_threads = 50;'
    if text.count(old) != 1:
        raise RuntimeError('Unexpected upstream Ceres solver; worker patch was not applied.')
    text = text.replace('#include <unordered_map>',
                        '#include <algorithm>\n#include <thread>\n#include <unordered_map>', 1)
    text = text.replace(old, 'options_.num_threads = std::max(1u, std::min(4u, '
                        'std::thread::hardware_concurrency()));', 1)
    solver.write_text(text)
    build = DEPS / 'build/slam_solver'
    env = tf2_environment()
    run(['cmake', '-S', str(WORKSPACE / 'runtime/slam_solver'), '-B', str(build),
         f'-DSOLVER_SOURCE={source}', '-DCMAKE_BUILD_TYPE=Release',
         f'-DCMAKE_INSTALL_PREFIX={runtime}'], env)
    run(['cmake', '--build', str(build), '--parallel', '2'], env)
    run(['cmake', '--install', str(build)], env)
    index = runtime / 'share/ament_index/resource_index'
    (index / 'packages').mkdir(parents=True, exist_ok=True)
    (index / 'packages/slam_toolbox').touch()
    plugins = index / 'slam_toolbox__pluginlib__plugin'
    plugins.mkdir(parents=True, exist_ok=True)
    shutil.copyfile('/opt/ros/humble/share/ament_index/resource_index/'
                    'slam_toolbox__pluginlib__plugin/slam_toolbox', plugins / 'slam_toolbox')
    for target, original in [(runtime / 'share/slam_toolbox', '/opt/ros/humble/share/slam_toolbox'),
                             (runtime / 'lib/slam_toolbox', '/opt/ros/humble/lib/slam_toolbox')]:
        if target.is_symlink():
            target.unlink()
        target.symlink_to(original, target_is_directory=True)
    marker.write_text(stamp + '\n')


def prepare_controller():
    original = Path('/opt/ros/humble/share/irobot_create_control')
    runtime = DEPS / 'control_runtime'
    share = runtime / 'share/irobot_create_control'
    (share / 'config').mkdir(parents=True, exist_ok=True)
    for item in original.iterdir():
        if item.name != 'config':
            target = share / item.name
            if not target.exists():
                target.symlink_to(item, target_is_directory=item.is_dir())
    for item in (original / 'config').iterdir():
        target = share / 'config' / item.name
        if item.name == 'control.yaml':
            if target.is_symlink():
                target.unlink()
            data = yaml.safe_load(item.read_text())
            data['/**']['controller_manager']['ros__parameters']['update_rate'] = 100
            target.write_text(yaml.safe_dump(data, sort_keys=False))
        elif not target.exists():
            target.symlink_to(item, target_is_directory=item.is_dir())
    index = runtime / 'share/ament_index/resource_index/packages'
    index.mkdir(parents=True, exist_ok=True)
    (index / 'irobot_create_control').touch()
    # Rendering the Standard robot at cold start can hold a controller service
    # longer than the spawner's 10 s default. Keep the official spawner/controller;
    # only give startup RPCs enough time, preventing duplicate configure requests.
    (index / 'controller_manager').touch()
    manager_share = runtime / 'share/controller_manager'
    if not manager_share.exists():
        manager_share.symlink_to('/opt/ros/humble/share/controller_manager', target_is_directory=True)
    manager_bin = runtime / 'lib/controller_manager'
    manager_bin.mkdir(parents=True, exist_ok=True)
    for item in Path('/opt/ros/humble/lib/controller_manager').iterdir():
        target = manager_bin / item.name
        if item.name != 'spawner' and not target.exists():
            target.symlink_to(item, target_is_directory=item.is_dir())
    spawner = manager_bin / 'spawner'
    spawner.write_text('''#!/usr/bin/env python3
import os
import sys
extra = []
for option, value in [('--service-call-timeout', '60'), ('--controller-manager-timeout', '120')]:
    if not any(arg.split('=')[0] == option for arg in sys.argv[1:]):
        extra.extend([option, value])
original = '/opt/ros/humble/lib/controller_manager/spawner'
os.execv(original, [original] + extra + sys.argv[1:])
''')
    spawner.chmod(0o755)
    print('Local Create 3 controller configuration: 100 Hz.', flush=True)


def main():
    if not os.environ.get('AMENT_PREFIX_PATH') or not shutil.which('colcon'):
        raise RuntimeError('Install dependencies and source /opt/ros/humble/setup.bash first.')
    prepare_tf2()
    prepare_solver()
    prepare_controller()
    print('Runtime preparation complete. Build turtlebot4_btth4 next.', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'Runtime preparation failed: {error}', file=sys.stderr)
        raise SystemExit(1)
