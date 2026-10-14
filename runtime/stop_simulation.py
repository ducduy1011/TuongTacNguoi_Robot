"""Stop only the selected BTTH04 domain/partition, shutting Nav2 down first."""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes


def processes(domain, partition):
    found = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            env = (entry / 'environ').read_bytes().split(b'\0')
            if f'ROS_DOMAIN_ID={domain}'.encode() not in env or f'IGN_PARTITION={partition}'.encode() not in env:
                continue
            args = (entry / 'cmdline').read_bytes().decode().split('\0')[:-1]
            launcher = any(args[i:i+2] == ['launch', 'turtlebot4_btth4'] for i in range(len(args)))
            executable = args[0] if args else ''
            node = executable.startswith('/opt/ros/humble/lib/') or executable.endswith('/teleop_wasd')
            gazebo = executable.startswith('ign gazebo')
            if launcher or node or gazebo:
                found[int(entry.name)] = launcher
        except (OSError, UnicodeError):
            continue
    return found


def send(pids, sig):
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def main():
    domain = os.environ.get('ROS_DOMAIN_ID', '')
    partition = os.environ.get('IGN_PARTITION', '')
    if not domain.isdigit() or not partition.startswith('turtlebot4_btth4'):
        raise SystemExit('Source the BTTH04 env.sh before stopping.')
    rclpy.init()
    node = rclpy.create_node('btth4_shutdown')
    try:
        for manager in ('lifecycle_manager_navigation', 'lifecycle_manager_localization'):
            client = node.create_client(ManageLifecycleNodes, f'/{manager}/manage_nodes')
            if client.wait_for_service(timeout_sec=2.0):
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.SHUTDOWN
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
            node.destroy_client(client)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    selected = processes(domain, partition)
    # Fortress' Ruby launcher does not forward SIGINT reliably to both children.
    # Stop the server through its own service before stopping the launch roots.
    gazebo = []
    gui = []
    servers = []
    for pid in selected:
        try:
            command = Path(f'/proc/{pid}/cmdline').read_bytes()
            if command.startswith(b'ign gazebo'):
                gazebo.append(pid)
            if command.startswith(b'ign gazebo server'):
                servers.append(pid)
            if command.startswith(b'ign gazebo gui'):
                gui.append(pid)
        except OSError:
            pass
    if gazebo:
        try:
            subprocess.run(['ign', 'service', '-s', '/server_control',
                            '--reqtype', 'ignition.msgs.ServerControl',
                            '--reptype', 'ignition.msgs.Boolean', '--timeout', '2000',
                            '--req', 'stop: true'], timeout=3,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            send(servers, signal.SIGINT)
            send(gui, signal.SIGINT)
            end = time.monotonic() + 5.0
            while time.monotonic() < end and any(Path(f'/proc/{p}').exists() for p in gazebo):
                time.sleep(0.2)
        except (OSError, subprocess.TimeoutExpired):
            pass
    send([p for p, launcher in selected.items() if launcher], signal.SIGINT)
    end = time.monotonic() + 8.0
    while time.monotonic() < end and processes(domain, partition):
        time.sleep(0.2)
    send(processes(domain, partition), signal.SIGTERM)
    end = time.monotonic() + 3.0
    while time.monotonic() < end and processes(domain, partition):
        time.sleep(0.2)
    send(processes(domain, partition), signal.SIGKILL)
    print(f'BTTH04 domain {domain}: stopped simulator, SLAM, localization, Nav2 and teleop.')


if __name__ == '__main__':
    main()
