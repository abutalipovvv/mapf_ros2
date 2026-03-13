from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib

if os.environ.get("DISPLAY"):
    matplotlib.use("TkAgg")
else:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

import yaml

from scripts.utils.landmark_loader import load_landmarks

WorldPoint = Tuple[float, float]
LandmarkEdge = Tuple[str, str]


class GridMonitorNode(Node):
    def __init__(self):
        super().__init__("grid_monitor_node")

        self.robot_names, self.initial_robot_poses = self._load_robot_config()
        self.declare_parameter("refresh_sec", 1.0)
        self.declare_parameter("save_png", True)
        self.declare_parameter("png_path", "/root/ws/src/output/fleet_live_monitor.png")
        self.declare_parameter("interactive", bool(os.environ.get("DISPLAY")))
        self.declare_parameter("path_history_len", 40)

        self.refresh_sec = max(0.2, float(self.get_parameter("refresh_sec").value))
        self.save_png = self._as_bool(self.get_parameter("save_png").value)
        self.interactive = self._as_bool(self.get_parameter("interactive").value)
        self.path_history_len = max(1, int(self.get_parameter("path_history_len").value))
        self.png_path = Path(str(self.get_parameter("png_path").value)).expanduser()

        self.robot_poses: Dict[str, Optional[WorldPoint]] = {
            name: self.initial_robot_poses.get(name) for name in self.robot_names
        }
        self.robot_paths: Dict[str, List[WorldPoint]] = {
            name: ([self.initial_robot_poses[name]] if name in self.initial_robot_poses else [])
            for name in self.robot_names
        }
        self.active_tasks: Dict[str, Tuple[str, str]] = {}
        self.robot_global_paths: Dict[str, List[WorldPoint]] = {
            name: [] for name in self.robot_names
        }
        self.robot_local_paths: Dict[str, List[WorldPoint]] = {
            name: [] for name in self.robot_names
        }
        self.robot_current_nav_goals: Dict[str, Optional[WorldPoint]] = {
            name: None for name in self.robot_names
        }

        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_path = share_dir / "config" / "landmarks.yaml"
        self.landmarks = load_landmarks(str(landmarks_path))
        self.landmark_edges = self._build_landmark_graph()
        self.view_bounds = self._compute_view_bounds()

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

        self.task_status_sub = self.create_subscription(
            String,
            "/fleet/task_status",
            self._task_status_callback,
            10,
        )
        self.visualization_sub = self.create_subscription(
            String,
            "/fleet/visualization",
            self._visualization_callback,
            10,
        )

        self.timer = self.create_timer(self.refresh_sec, self._tick)

        self.fig = None
        self.ax = None
        if self.interactive:
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10, 8))
            manager = getattr(self.fig.canvas, "manager", None)
            if manager is not None and hasattr(manager, "set_window_title"):
                manager.set_window_title("Fleet Visualizer")
            plt.show(block=False)

        self.get_logger().info("GridMonitorNode started")
        self.get_logger().info(f"Robots: {self.robot_names}")
        self.get_logger().info(
            f"Visualizer: interactive={self.interactive}, save_png={self.save_png}, png_path={self.png_path}"
        )
        self.get_logger().info(f"Matplotlib backend: {matplotlib.get_backend()}")

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _load_robot_config(self) -> Tuple[List[str], Dict[str, WorldPoint]]:
        fallback = ["robot1", "robot2"]
        fallback_positions: Dict[str, WorldPoint] = {}
        try:
            gazebo_share = Path(get_package_share_directory("gazebo_sim"))
            robots_path = gazebo_share / "config" / "robots.yaml"
            if not robots_path.exists():
                return fallback, fallback_positions

            data = yaml.safe_load(robots_path.read_text(encoding="utf-8")) or {}
            robots_raw = data.get("robots", [])
            parsed: List[str] = []
            positions: Dict[str, WorldPoint] = {}
            if isinstance(robots_raw, list):
                for item in robots_raw:
                    if not isinstance(item, dict):
                        continue
                    name = item.get("name")
                    if isinstance(name, str):
                        clean = name.strip().strip("/")
                        if clean:
                            parsed.append(clean)
                            try:
                                positions[clean] = (
                                    float(item.get("x_pose", 0.0)),
                                    float(item.get("y_pose", 0.0)),
                                )
                            except (TypeError, ValueError):
                                pass
            parsed = list(dict.fromkeys(parsed))
            if parsed:
                return parsed, positions
            return fallback, fallback_positions
        except Exception:
            return fallback, fallback_positions

    def _build_landmark_graph(self) -> List[LandmarkEdge]:
        grouped_by_y: Dict[float, List[Tuple[float, str]]] = {}
        grouped_by_x: Dict[float, List[Tuple[float, str]]] = {}

        for name, lm in self.landmarks.items():
            x = round(float(lm["x"]), 4)
            y = round(float(lm["y"]), 4)
            grouped_by_y.setdefault(y, []).append((x, name))
            grouped_by_x.setdefault(x, []).append((y, name))

        edge_set = set()
        for items in grouped_by_y.values():
            items.sort()
            for idx in range(len(items) - 1):
                edge_set.add(tuple(sorted((items[idx][1], items[idx + 1][1]))))

        for items in grouped_by_x.values():
            items.sort()
            for idx in range(len(items) - 1):
                edge_set.add(tuple(sorted((items[idx][1], items[idx + 1][1]))))

        return sorted(edge_set)

    def _compute_view_bounds(self) -> Tuple[float, float, float, float]:
        points: List[WorldPoint] = []
        for lm in self.landmarks.values():
            points.append((float(lm["x"]), float(lm["y"])))

        if not points:
            return (-5.0, 5.0, -5.0, 5.0)

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        margin = 1.0
        return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)

    def _pose_callback(self, robot_name: str, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        pose = (float(p.x), float(p.y))
        self.robot_poses[robot_name] = pose

        history = self.robot_paths.setdefault(robot_name, [])
        history.append(pose)
        if len(history) > self.path_history_len:
            del history[:-self.path_history_len]

    def _task_status_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        robot = data.get("robot")
        goal = data.get("goal")
        state = data.get("state")
        if not isinstance(robot, str) or not isinstance(goal, str) or not isinstance(state, str):
            return

        if state in {"accepted", "planned", "executing", "requeued", "canceling"}:
            self.active_tasks[robot] = (goal, state)
        elif state in {"completed", "failed", "canceled"}:
            active = self.active_tasks.get(robot)
            if active is not None and active[0] == goal:
                self.active_tasks.pop(robot, None)

    @staticmethod
    def _payload_to_point(data: object) -> Optional[WorldPoint]:
        if not isinstance(data, dict):
            return None
        x = data.get("x")
        y = data.get("y")
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return None
        return float(x), float(y)

    def _visualization_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        robots = data.get("robots")
        if not isinstance(robots, dict):
            return

        for robot_name, robot_data in robots.items():
            if robot_name not in self.robot_poses or not isinstance(robot_data, dict):
                continue

            pose = self._payload_to_point(robot_data.get("pose"))
            if pose is not None:
                self.robot_poses[robot_name] = pose

            current_nav_goal = self._payload_to_point(robot_data.get("current_nav_goal"))
            self.robot_current_nav_goals[robot_name] = current_nav_goal

            local_path_raw = robot_data.get("local_path")
            if isinstance(local_path_raw, list):
                self.robot_local_paths[robot_name] = [
                    point
                    for point in (self._payload_to_point(item) for item in local_path_raw)
                    if point is not None
                ]

            global_path_raw = robot_data.get("global_path")
            if isinstance(global_path_raw, list):
                self.robot_global_paths[robot_name] = [
                    point
                    for point in (self._payload_to_point(item) for item in global_path_raw)
                    if point is not None
                ]

    def _robot_color(self, robot_name: str, active: bool) -> str:
        active_colors = {
            "robot1": "#2a9d8f",
            "robot2": "#e76f51",
            "robot3": "#264653",
            "robot4": "#e9c46a",
        }
        if active:
            return active_colors.get(robot_name, "#1d3557")
        return "#495057"

    def _nearest_landmark_name(self, point: WorldPoint, max_distance: float = 0.9) -> Optional[str]:
        best_name: Optional[str] = None
        best_distance_sq = max_distance * max_distance
        px, py = point
        for name, lm in self.landmarks.items():
            dx = float(lm["x"]) - px
            dy = float(lm["y"]) - py
            dist_sq = dx * dx + dy * dy
            if dist_sq <= best_distance_sq:
                best_distance_sq = dist_sq
                best_name = name
        return best_name

    def _landmark_sequence_from_points(self, points: List[WorldPoint]) -> List[str]:
        sequence: List[str] = []
        for point in points:
            name = self._nearest_landmark_name(point)
            if name is None:
                continue
            if not sequence or sequence[-1] != name:
                sequence.append(name)
        return sequence

    def _draw_graph_edges(self, ax, sequence: List[str], color: str, linewidth: float, alpha: float, zorder: int) -> None:
        if len(sequence) < 2:
            return
        for idx in range(len(sequence) - 1):
            a = self.landmarks.get(sequence[idx])
            b = self.landmarks.get(sequence[idx + 1])
            if a is None or b is None:
                continue
            ax.plot(
                [float(a["x"]), float(b["x"])],
                [float(a["y"]), float(b["y"])],
                color=color,
                linewidth=linewidth,
                alpha=alpha,
                zorder=zorder,
            )

    def _draw(self, ax) -> None:
        x_min, x_max, y_min, y_max = self.view_bounds

        ax.clear()
        ax.set_title("Fleet Visualizer")
        ax.set_facecolor("#f8f9fa")
        ax.grid(True, color="#dee2e6", linewidth=0.8, alpha=0.9)
        ax.set_aspect("equal")

        for a_name, b_name in self.landmark_edges:
            a = self.landmarks.get(a_name)
            b = self.landmarks.get(b_name)
            if a is None or b is None:
                continue
            ax.plot(
                [float(a["x"]), float(b["x"])],
                [float(a["y"]), float(b["y"])],
                color="#adb5bd",
                linewidth=2.0,
                alpha=0.8,
                zorder=1,
            )

        active_goals = {goal for goal, _ in self.active_tasks.values()}
        for lm_name, lm in self.landmarks.items():
            x = float(lm["x"])
            y = float(lm["y"])
            is_active = lm_name in active_goals
            face_color = "#ffb703" if is_active else "#ffffff"
            edge_color = "#fb8500" if is_active else "#4361ee"
            ax.scatter(
                x,
                y,
                s=260,
                marker="o",
                c=face_color,
                edgecolors=edge_color,
                linewidths=2.0,
                zorder=3,
            )
            ax.text(x + 0.12, y + 0.12, lm_name, fontsize=9, color=edge_color, zorder=4)

        for robot_name in self.robot_names:
            pose = self.robot_poses.get(robot_name)
            active = robot_name in self.active_tasks
            color = self._robot_color(robot_name, active)

            history = self.robot_paths.get(robot_name, [])
            if len(history) >= 2:
                ax.plot(
                    [p[0] for p in history],
                    [p[1] for p in history],
                    linestyle="--",
                    linewidth=1.8,
                    color=color,
                    alpha=0.65,
                    zorder=2,
                )

            if pose is None:
                continue

            global_path = self.robot_global_paths.get(robot_name, [])
            if global_path:
                global_sequence = self._landmark_sequence_from_points(global_path)
                self._draw_graph_edges(
                    ax,
                    global_sequence,
                    color=color,
                    linewidth=5.5,
                    alpha=0.28,
                    zorder=2,
                )

            ax.scatter(
                pose[0],
                pose[1],
                marker="o",
                s=320,
                c="white",
                edgecolors="black",
                linewidths=1.8,
                zorder=5,
            )
            ax.scatter(
                pose[0],
                pose[1],
                marker="o",
                s=180,
                c=color,
                edgecolors="black",
                linewidths=1.2,
                zorder=6,
            )

            label = f"{robot_name} [idle]"
            active_task = self.active_tasks.get(robot_name)
            if active_task is not None:
                goal_name, state = active_task
                label = f"{robot_name} [{state}]"
                lm = self.landmarks.get(goal_name)
                if lm is not None:
                    goal_x = float(lm["x"])
                    goal_y = float(lm["y"])
                    ax.scatter(
                        goal_x,
                        goal_y,
                        s=340,
                        marker="o",
                        facecolors="none",
                        edgecolors=color,
                        linewidths=2.2,
                        zorder=4,
                    )
                    ax.text(
                        goal_x + 0.12,
                        goal_y - 0.22,
                        f"target {goal_name}",
                        fontsize=8,
                        color=color,
                        zorder=6,
                    )

            ax.text(
                pose[0] + 0.14,
                pose[1] - 0.20,
                label,
                fontsize=9,
                color="black",
                bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.75, "pad": 1.5},
                zorder=7,
            )

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_xlabel("world x")
        ax.set_ylabel("world y")

    def _save_png(self) -> None:
        self.png_path.parent.mkdir(parents=True, exist_ok=True)
        fig = Figure(figsize=(10, 8))
        FigureCanvasAgg(fig)
        ax = fig.add_subplot(111)
        self._draw(ax)
        fig.tight_layout()
        fig.savefig(self.png_path, dpi=150)

    def _update_window(self) -> None:
        if not self.interactive or self.fig is None or self.ax is None:
            return

        self._draw(self.ax)
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def _tick(self) -> None:
        if self.save_png:
            self._save_png()
        self._update_window()


def main(args=None):
    rclpy.init(args=args)
    node = GridMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        plt.close("all")
        rclpy.shutdown()
