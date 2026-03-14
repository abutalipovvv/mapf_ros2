from __future__ import annotations

import argparse
import json
import time
from typing import List, Optional
from uuid import uuid4

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TaskManagerNode(Node):
    def __init__(self, robot: str, wait_status: bool = False):
        super().__init__("task_manager")
        self.robot = robot
        self.pub = self.create_publisher(String, "/fleet/tasks", 10)
        self.wait_status = wait_status

        self._waiting_goal: Optional[str] = None
        self._waiting_task_id: Optional[str] = None
        self._waiting_done = False
        self._waiting_state: Optional[str] = None

        if self.wait_status:
            self.status_sub = self.create_subscription(
                String,
                "/fleet/task_status",
                self._status_callback,
                10,
            )
        else:
            self.status_sub = None

    def send_task(
        self,
        goal: Optional[str],
        task_id: Optional[str] = None,
        command: str = "task",
        reason: Optional[str] = None,
    ):
        if self.wait_status and goal is not None:
            self._waiting_goal = goal
            self._waiting_task_id = task_id
            self._waiting_done = False
            self._waiting_state = None
        elif self.wait_status:
            self._waiting_goal = None
            self._waiting_task_id = None
            self._waiting_done = False
            self._waiting_state = None

        msg = String()
        payload = {
            "robot": self.robot,
            "command": command,
        }
        if goal is not None:
            payload["goal"] = goal
        if task_id is not None:
            payload["task_id"] = task_id
        if reason:
            payload["reason"] = reason
        msg.data = json.dumps(payload)

        self.get_logger().info(f"Publishing task: {msg.data}")
        self.pub.publish(msg)

    def _status_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        robot = data.get("robot")
        goal = data.get("goal")
        state = data.get("state")
        task_id = data.get("task_id")
        if not isinstance(robot, str) or not isinstance(goal, str) or not isinstance(state, str):
            return

        if robot != self.robot:
            return

        if self._waiting_goal is None:
            return

        # Prefer strict correlation by task_id when present.
        if isinstance(self._waiting_task_id, str) and self._waiting_task_id:
            if isinstance(task_id, str):
                if task_id != self._waiting_task_id:
                    return
            elif goal != self._waiting_goal:
                return
        elif goal != self._waiting_goal:
            return

        if state in {"completed", "failed"}:
            self._waiting_state = state
            self._waiting_done = True

    def wait_for_terminal_status(self, timeout_sec: float) -> Optional[str]:
        if not self.wait_status or self._waiting_goal is None:
            return None

        start_t = time.monotonic()
        while rclpy.ok():
            if self._waiting_done:
                return self._waiting_state

            if time.monotonic() - start_t >= timeout_sec:
                return None

            rclpy.spin_once(self, timeout_sec=0.1)

        return None


def _parse_goals(raw_goals: List[str]) -> List[str]:
    parsed: List[str] = []
    for item in raw_goals:
        for token in item.split(","):
            goal = token.strip()
            if goal:
                parsed.append(goal)
    return parsed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", required=True, help="Robot namespace, e.g. robot1")
    parser.add_argument(
        "--goal",
        action="append",
        help="Goal ID, e.g. LM4. You can repeat --goal or pass comma-separated goals.",
    )
    parser.add_argument(
        "--cancel",
        action="store_true",
        help="Send cancel command for current active task of robot.",
    )
    parser.add_argument(
        "--delay-sec",
        type=float,
        default=0.25,
        help="Delay between sequential task publishes.",
    )
    parser.add_argument(
        "--wait-status",
        action="store_true",
        help="Wait for terminal status (completed/failed) before sending next goal.",
    )
    parser.add_argument(
        "--wait-timeout-sec",
        type=float,
        default=300.0,
        help="Timeout while waiting for terminal status for each goal.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="If set, continue sending next goals even when one goal failed.",
    )
    parser.add_argument(
        "--reason",
        default="",
        help="Optional reason string for cancel/task command.",
    )
    args = parser.parse_args()

    mode = "cancel" if args.cancel else "task"
    goals = _parse_goals(args.goal or [])
    if mode != "cancel" and not goals:
        raise ValueError("No valid goals provided.")
    if mode == "cancel" and goals:
        raise ValueError("--cancel does not use --goal.")

    rclpy.init()
    node = TaskManagerNode(args.robot, wait_status=args.wait_status)
    sequence_start_t = time.monotonic()
    completed_goals = 0

    # Чуть подождать, чтобы publisher успел зарегистрироваться
    time.sleep(0.5)
    if mode == "cancel":
        task_id = f"{args.robot}-cancel-{uuid4().hex[:8]}"
        node.send_task(
            goal=None,
            task_id=task_id,
            command="cancel",
            reason=args.reason or "user_cancel",
        )
        time.sleep(0.5)
        node.destroy_node()
        rclpy.shutdown()
        return

    for i, goal in enumerate(goals):
        task_id = f"{args.robot}-{i + 1}-{uuid4().hex[:8]}"
        goal_start_t = time.monotonic()
        node.send_task(
            goal=goal,
            task_id=task_id,
            command=mode,
            reason=args.reason if args.reason else None,
        )

        if args.wait_status:
            result_state = node.wait_for_terminal_status(timeout_sec=max(1.0, args.wait_timeout_sec))
            if result_state is None:
                node.get_logger().error(
                    f"Timeout waiting task status for goal={goal} "
                    f"(timeout={args.wait_timeout_sec:.1f}s)"
                )
                break

            elapsed = time.monotonic() - goal_start_t
            node.get_logger().info(f"Goal {goal} terminal status: {result_state}")
            node.get_logger().info(
                f"Goal {goal} finished in {elapsed:.2f}s"
            )
            if result_state == "completed":
                completed_goals += 1
            if result_state == "failed" and not args.continue_on_failure:
                node.get_logger().warn(
                    "Stopping sequence because goal failed "
                    "(use --continue-on-failure to continue)."
                )
                break

        if i < len(goals) - 1:
            time.sleep(max(0.0, args.delay_sec))
    if mode == "task" and args.wait_status:
        total_elapsed = time.monotonic() - sequence_start_t
        node.get_logger().info(
            f"Sequence finished: completed {completed_goals}/{len(goals)} goals "
            f"in {total_elapsed:.2f}s"
        )
    time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
