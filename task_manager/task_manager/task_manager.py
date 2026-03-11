from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskManagerNode(Node):
    def __init__(self, robot: str, goal: str):
        super().__init__("task_manager")
        self.robot = robot
        self.goal = goal
        self.pub = self.create_publisher(String, "/fleet/tasks", 10)

    def send_task(self):
        msg = String()
        msg.data = json.dumps({
            "robot": self.robot,
            "goal": self.goal,
        })

        self.get_logger().info(f"Publishing task: {msg.data}")
        self.pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", required=True, help="Robot namespace, e.g. robot1")
    parser.add_argument("--goal", required=True, help="Goal ID, e.g. LM4")
    args = parser.parse_args()

    rclpy.init()
    node = TaskManagerNode(args.robot, args.goal)

    # Чуть подождать, чтобы publisher успел зарегистрироваться
    time.sleep(0.5)
    node.send_task()
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()