from __future__ import annotations

import argparse
import math
from pathlib import Path
import time
from typing import Dict, List

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
import yaml


def _load_landmarks(yaml_path: str) -> Dict[str, Dict[str, float]]:
    path = Path(yaml_path)
    if not path.exists():
        raise FileNotFoundError(f"Landmarks file not found: {yaml_path}")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise ValueError("Landmarks YAML must contain a top-level dictionary")

    landmarks: Dict[str, Dict[str, float]] = {}
    for name, value in data.items():
        if not isinstance(name, str) or not isinstance(value, dict):
            continue
        if "x" not in value or "y" not in value:
            continue
        landmarks[name] = {
            "x": float(value["x"]),
            "y": float(value["y"]),
            "yaw": float(value.get("yaw", 0.0)),
        }
    return landmarks


def _yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def _parse_goals(raw_goals: List[str]) -> List[str]:
    parsed: List[str] = []
    for item in raw_goals:
        for token in item.split(","):
            goal = token.strip()
            if goal:
                parsed.append(goal)
    return parsed


class Nav2GoalSender(Node):
    def __init__(self, robot_name: str, landmarks: Dict[str, Dict[str, float]]) -> None:
        super().__init__("send_nav2_goals")
        self.robot_name = robot_name.strip().strip("/")
        self.landmarks = landmarks
        self.client = ActionClient(
            self,
            NavigateToPose,
            f"/{self.robot_name}/navigate_to_pose",
        )

    def send_goal(self, landmark_name: str, timeout_sec: float) -> float | None:
        landmark = self.landmarks.get(landmark_name)
        if landmark is None:
            raise KeyError(f"Unknown landmark: {landmark_name}")

        if not self.client.wait_for_server(timeout_sec=max(1.0, timeout_sec)):
            self.get_logger().error(
                f"NavigateToPose server not available for /{self.robot_name}/navigate_to_pose"
            )
            return None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = landmark["x"]
        goal_msg.pose.pose.position.y = landmark["y"]
        qz, qw = _yaw_to_quaternion(landmark["yaw"])
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"[{self.robot_name}] Sending Nav2 goal {landmark_name}: "
            f"x={landmark['x']:.3f}, y={landmark['y']:.3f}, yaw={landmark['yaw']:.3f}"
        )

        start_t = time.monotonic()
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        if not send_future.done():
            self.get_logger().error(f"[{self.robot_name}] Timed out waiting goal response")
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"[{self.robot_name}] Goal {landmark_name} was rejected")
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        if not result_future.done():
            self.get_logger().error(f"[{self.robot_name}] Timed out waiting result")
            return None

        result_wrap = result_future.result()
        if result_wrap is None:
            self.get_logger().error(f"[{self.robot_name}] Result missing")
            return None

        status = int(result_wrap.status)
        if status == 4:
            elapsed = time.monotonic() - start_t
            self.get_logger().info(
                f"[{self.robot_name}] Goal {landmark_name} succeeded in {elapsed:.2f}s"
            )
            return elapsed

        self.get_logger().error(
            f"[{self.robot_name}] Goal {landmark_name} finished with status={status}"
        )
        return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", required=True, help="Robot namespace, e.g. robot1")
    parser.add_argument(
        "--goal",
        action="append",
        required=True,
        help="Landmark name, e.g. LM4. Repeat --goal or pass comma-separated values.",
    )
    parser.add_argument(
        "--landmarks-file",
        default="",
        help="Optional absolute path to landmarks.yaml. By default uses hybrid_fleet_manager config.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=300.0,
        help="Timeout for goal response and result waiting.",
    )
    args = parser.parse_args()

    goals = _parse_goals(args.goal or [])
    if not goals:
        raise ValueError("No valid goals provided.")

    landmarks_file = args.landmarks_file.strip()
    if not landmarks_file:
        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_file = str(share_dir / "config" / "landmarks.yaml")

    landmarks = _load_landmarks(landmarks_file)

    rclpy.init()
    node = Nav2GoalSender(args.robot, landmarks)
    total_start_t = time.monotonic()
    completed_goals = 0
    try:
        for landmark_name in goals:
            elapsed = node.send_goal(landmark_name, timeout_sec=max(1.0, args.timeout_sec))
            if elapsed is None:
                break
            completed_goals += 1
        total_elapsed = time.monotonic() - total_start_t
        node.get_logger().info(
            f"[{node.robot_name}] Completed {completed_goals}/{len(goals)} goals "
            f"in {total_elapsed:.2f}s"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
