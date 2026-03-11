from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from hybrid_fleet_manager.landmark_loader import load_landmarks


class GridMonitorNode(Node):
    def __init__(self):
        super().__init__("grid_monitor_node")

        self.robot_names = ["robot1", "robot2"]
        self.robot_poses: Dict[str, Optional[Tuple[float, float]]] = {
            name: None for name in self.robot_names
        }

        self.map_msg: Optional[OccupancyGrid] = None
        self.map_data: Optional[List[int]] = None
        self.last_saved_path = Path("/root/ws/src/hybrid_fleet_manager/mapf_grid_monitor.png")

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/robot1/map",
            self._map_callback,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        amcl_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pose_subs = []
        for name in self.robot_names:
            sub = self.create_subscription(
                PoseWithCovarianceStamped,
                f"/{name}/amcl_pose",
                lambda msg, robot_name=name: self._pose_callback(robot_name, msg),
                amcl_qos,
            )
            self.pose_subs.append(sub)

        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_path = share_dir / "config" / "landmarks.yaml"
        self.landmarks = load_landmarks(str(landmarks_path))

        

        self.get_logger().info("GridMonitorNode started")
        self.get_logger().info(f"PNG output: {self.last_saved_path}")
        self._save_png()
        self.get_logger().info(f"Saved PNG: {self.last_saved_path}")

    def _map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_data = list(msg.data)

    def _pose_callback(self, robot_name: str, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.robot_poses[robot_name] = (p.x, p.y)

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        if self.map_msg is None:
            return None

        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        resolution = self.map_msg.info.resolution
        width = self.map_msg.info.width
        height = self.map_msg.info.height

        i = int(math.floor((x - origin_x) / resolution))
        j = int(math.floor((y - origin_y) / resolution))

        if i < 0 or j < 0 or i >= width or j >= height:
            return None
        return i, j

    def is_obstacle(self, i: int, j: int) -> bool:
        if self.map_msg is None or self.map_data is None:
            return True

        width = self.map_msg.info.width
        idx = j * width + i
        if idx < 0 or idx >= len(self.map_data):
            return True

        value = self.map_data[idx]
        return value == -1 or value >= 50

    def compute_simple_congestion(self) -> List[List[float]]:
        if self.map_msg is None:
            return []

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        heat = [[0.0 for _ in range(width)] for _ in range(height)]

        for pose in self.robot_poses.values():
            if pose is None:
                continue

            cell = self.world_to_grid(pose[0], pose[1])
            if cell is None:
                continue

            rx, ry = cell
            for j in range(max(0, ry - 5), min(height, ry + 6)):
                for i in range(max(0, rx - 5), min(width, rx + 6)):
                    dist = math.sqrt((i - rx) ** 2 + (j - ry) ** 2)
                    score = max(0.0, 6.0 - dist)
                    heat[j][i] += score

        max_val = max((max(row) for row in heat), default=1.0)
        if max_val < 1e-6:
            max_val = 1.0

        for j in range(height):
            for i in range(width):
                heat[j][i] /= max_val

        return heat

    def _save_png(self):
        if self.map_msg is None or self.map_data is None:
            return

        fig, ax = plt.subplots(figsize=(10, 8))
        ax.set_title("MAPF Grid Monitor (simplified)")
        ax.set_aspect("equal")

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        heat = self.compute_simple_congestion()

        for j in range(height):
            for i in range(width):
                if self.is_obstacle(i, j):
                    color = (0.15, 0.15, 0.15, 1.0)
                else:
                    c = heat[j][i] if heat else 0.0
                    color = (1.0, 1.0 - 0.6 * c, 1.0 - 0.9 * c, 1.0)

                rect = patches.Rectangle(
                    (i, j),
                    1,
                    1,
                    facecolor=color,
                    edgecolor=(0.8, 0.8, 0.8, 0.15),
                    linewidth=0.2,
                )
                ax.add_patch(rect)

        for lm_name, lm in self.landmarks.items():
            cell = self.world_to_grid(lm["x"], lm["y"])
            if cell is None:
                continue
            i, j = cell
            ax.scatter(i + 0.5, j + 0.5, marker="*", s=120, c="blue")
            ax.text(i + 0.6, j + 0.6, lm_name, fontsize=8, color="blue")

        colors = {
            "robot1": "green",
            "robot2": "red",
            "robot3": "purple",
        }

        for name, pose in self.robot_poses.items():
            if pose is None:
                continue

            cell = self.world_to_grid(pose[0], pose[1])
            if cell is None:
                continue

            i, j = cell
            ax.scatter(i + 0.5, j + 0.5, marker="o", s=100, c=colors.get(name, "green"))
            ax.text(i + 0.6, j + 0.2, name, fontsize=8, color="black")

        ax.set_xlim(0, width)
        ax.set_ylim(0, height)
        ax.set_xlabel("grid i")
        ax.set_ylabel("grid j")
        ax.invert_yaxis()
        fig.tight_layout()
        fig.savefig(self.last_saved_path, dpi=150)
        plt.close(fig)

    def _tick(self):
        if self.map_msg is None:
            self.get_logger().info("Waiting for /robot1/map ...")
            return

        self._save_png()
        self.get_logger().info(f"Saved PNG: {self.last_saved_path}")


def main(args=None):
    rclpy.init(args=args)
    node = GridMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()