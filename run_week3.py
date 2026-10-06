#!/usr/bin/env python3
"""Load the Week 3 environment and run one component in this terminal."""
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
PACKAGE = 'ur3_llm_control_week3'
COMMANDS = {
    'sim': ['ros2', 'launch', PACKAGE, 'sim_moveit_week3.launch.py', 'gui:=true', 'rviz:=true'],
    'camera': ['ros2', 'launch', PACKAGE, 'camera_week3.launch.py'],
    'skills': ['ros2', 'run', PACKAGE, 'skill_server', '--ros-args', '-p', 'use_sim_time:=true'],
    'planner': ['ros2', 'run', PACKAGE, 'llm_task.py', '--ros-args', '-p', 'use_sim_time:=true'],
    'cli': ['ros2', 'run', PACKAGE, 'user_command_cli.py'],
}


def main():
    role = sys.argv[1] if len(sys.argv) > 1 else ''
    if role not in (*COMMANDS, 'router'):
        raise SystemExit('Dùng: run_week3.py sim|camera|skills|router|planner|cli')
    os.chdir(ROOT)
    if role == 'router':
        os.execv(str(ROOT / 'run_9router.bash'), ['run_9router.bash', *sys.argv[2:]])
    setup = 'source "$1/setup_week3.bash"\n'
    if role == 'planner':
        setup += 'source "$1/private/ninerouter.env"\n'
    setup += 'shift\nexec "$@"'
    os.execv('/bin/bash', ['bash', '-e', '-c', setup, 'week3', str(ROOT),
                          *COMMANDS[role], *sys.argv[2:]])


if __name__ == '__main__':
    main()
