#!/usr/bin/env python3
import argparse
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description='Send a natural-language task to the Week 3 planner')
    parser.add_argument('--command', help='Send one command; otherwise use interactive input')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = rclpy.create_node('user_command_cli')
    publisher = node.create_publisher(String, '/user_command', 10)
    try:
        deadline = time.monotonic()+10
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not publisher.get_subscription_count():
            raise SystemExit('LLM planner is unavailable')
        while rclpy.ok():
            command = (args.command if args.command is not None else input('COMMAND > ')).strip()
            if command.lower() in ('exit', 'quit'):
                break
            if command:
                publisher.publish(String(data=command))
                print('SENT:', command, flush=True)
                if not publisher.wait_for_all_acked(timeout=Duration(seconds=5)):
                    raise SystemExit('Planner did not acknowledge the command')
            if args.command is not None:
                break
    except (KeyboardInterrupt, EOFError, ExternalShutdownException):
        pass
    except Exception:
        if node.context.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
