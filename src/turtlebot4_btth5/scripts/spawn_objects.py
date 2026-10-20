#!/usr/bin/env python3
"""Call Fortress create services and check the actual Boolean response."""
import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import time


SPECS = [('btth5_person', 'Standing_person', -6.0, -0.5, -2.15),
         ('btth5_chair', 'Chair', -6.3, -6.6, -0.82),
         ('btth5_person_2', 'Standing_person', -8.5, 2.0, 0.0),
         ('btth5_chair_2', 'Chair', -7.5, 3.5, -1.57),
         ('btth5_person_3', 'Standing_person', -2.0, -8.8, 1.57),
         ('btth5_chair_3', 'Chair', -0.2, -8.8, 1.57),
         ('btth5_person_4', 'Standing_person', 6.0, -7.3, 3.14),
         ('btth5_chair_4', 'Chair', 8.5, -6.6, 3.14)]


def wait_for_controllers(timeout):
    # Do not load eight textured meshes while wheel controllers are configuring.
    import rclpy
    from controller_manager_msgs.srv import ListControllers
    rclpy.init()
    node = rclpy.create_node('wait_before_loading_scene')
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    print('Waiting for both wheel-controller components before loading objects.', flush=True)
    deadline = time.monotonic() + timeout
    future = None
    next_poll = 0.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if future is None and time.monotonic() >= next_poll and client.service_is_ready():
                future = client.call_async(ListControllers.Request())
                next_poll = time.monotonic() + 1.0
            if future is not None and future.done():
                states = {c.name: c.state for c in future.result().controller}
                if all(states.get(n) == 'active' for n in ('joint_state_broadcaster', 'diffdrive_controller')):
                    return
                future = None
        raise RuntimeError('Wheel controllers are not active; objects were not created.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def create_objects(world, models, wait_seconds=0.0):
    paths = [models / folder / 'model.sdf' for _, folder, *_ in SPECS]
    for path in paths:
        if not path.is_file():
            raise RuntimeError('Missing model. Run python3 tools/prepare_scene.py first.')
    service = f'/world/{world}/create'
    deadline = time.monotonic() + wait_seconds
    while True:
        try:
            listed = subprocess.run(['ign', 'service', '-l'], capture_output=True, text=True,
                                    timeout=8, check=True)
            if service in listed.stdout.split():
                break
        except subprocess.SubprocessError:
            pass
        if time.monotonic() >= deadline:
            raise RuntimeError(f'{service} is not available. Start simulation.launch.py first, '
                               'wait for Gazebo, and source install/setup.bash in this terminal.')
        time.sleep(0.5)
    for (name, _, x, y, yaw), path in zip(SPECS, paths):
        request = (f'sdf_filename: {json.dumps(str(path.resolve()))} '
                   f'name: {json.dumps(name)} allow_renaming: false '
                   f'pose {{ position {{ x: {x} y: {y} z: 0 }} '
                   f'orientation {{ z: {math.sin(yaw/2)} w: {math.cos(yaw/2)} }} }}')
        response = subprocess.run([
            'ign', 'service', '-s', service, '--reqtype', 'ignition.msgs.EntityFactory',
            '--reptype', 'ignition.msgs.Boolean', '--timeout', '10000', '--req', request],
            capture_output=True, text=True, timeout=15)
        if response.returncode != 0 or not re.search(r'\bdata:\s*true\b', response.stdout):
            raise RuntimeError(f'Could not create {name} in {world}. '
                               'Check Gazebo/Play; the name may already exist. '
                               f'Response: {(response.stdout + response.stderr).strip()[-400:]}')
        print(f'Created {name} in world {world} at ({x}, {y}, 0).', flush=True)
    print(f'All {len(SPECS)} objects accepted by Gazebo. Find btth5_* in Entity Tree.',
          flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--models', type=Path, required=True)
    parser.add_argument('--wait', type=float, default=90.0)
    parser.add_argument('--wait-controllers', action='store_true')
    # launch_ros appends --ros-args; this helper has no ROS node or motion command.
    args, _ = parser.parse_known_args()
    try:
        if args.wait_controllers:
            wait_for_controllers(args.wait)
        create_objects(args.world, args.models.expanduser(), args.wait)
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f'[spawn_objects] ERROR: {error}', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
