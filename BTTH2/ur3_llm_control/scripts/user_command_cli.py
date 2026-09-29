#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class UserCommandCLI(Node):

    def __init__(self):
        super().__init__("user_command_cli")

        self.publisher = self.create_publisher(
            String,
            "/user_command",
            10
        )

        print()
        print("=" * 60)
        print("UR3e NATURAL LANGUAGE COMMAND")
        print("=" * 60)
        print("Nhập câu lệnh tự nhiên cho robot.")
        print("Gõ 'exit' hoặc 'quit' để thoát.")
        print("=" * 60)
        print()

    def run(self):

        # Cho ROS một chút thời gian tìm subscriber
        time.sleep(1.0)

        while rclpy.ok():

            try:
                command = input("COMMAND > ").strip()

            except (KeyboardInterrupt, EOFError):
                print()
                break

            if not command:
                continue

            if command.lower() in {
                "exit",
                "quit",
                "q"
            }:
                break

            msg = String()
            msg.data = command

            self.publisher.publish(msg)

            print(
                f"[SENT] {command}"
            )
            print()

            # Cho DDS xử lý publish
            rclpy.spin_once(
                self,
                timeout_sec=0.1
            )


def main(args=None):

    rclpy.init(args=args)

    node = UserCommandCLI()

    try:
        node.run()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
