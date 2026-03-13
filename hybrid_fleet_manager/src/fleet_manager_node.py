from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from scripts.planning.cbs_planner import CBSPlanner, RobotRequest
from scripts.planning.path_utils import compress_grid_path
from scripts.planning.search_utils import find_nearest_free_cell
from scripts.runtime.map_provider import MapProvider
from scripts.runtime.robot_client import RobotClient
from scripts.utils.geometry_utils import yaw_to_quaternion
from scripts.utils.grid_utils import grid_to_world, world_to_grid
from scripts.utils.landmark_loader import load_landmarks

GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]
TaskRef = Tuple[str, Optional[str]]  # (goal_name, task_id)


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__("fleet_manager_node")

        self.robot_names = self._load_robot_names()

        default_map_topic = "/robot1/map"
        self.declare_parameter("map_topic", default_map_topic)
        raw_map_topic = self.get_parameter("map_topic").value
        self.map_topic = (
            raw_map_topic if isinstance(raw_map_topic, str) and raw_map_topic else default_map_topic
        )

        self.declare_parameter("map_occ_threshold", 50)
        self.declare_parameter("map_unknown_is_free", True)
        self.map_occ_threshold = max(0, min(100, int(self.get_parameter("map_occ_threshold").value)))
        self.map_unknown_is_free = self._as_bool(self.get_parameter("map_unknown_is_free").value)

        self.declare_parameter("reservation_time_step_sec", 1.0)
        self.declare_parameter("nominal_robot_speed_mps", 0.30)
        self.declare_parameter("reservation_hold_sec", 12.0)
        self.declare_parameter("waypoint_timeout_sec", 45.0)
        self.declare_parameter("offpath_max_cell_distance", 2)
        self.declare_parameter("offpath_grace_ticks", 5)
        self.declare_parameter("max_requeue_attempts", 5)
        self.declare_parameter("cancel_timeout_sec", 2.0)
        self.declare_parameter("pending_blocked_timeout_sec", 120.0)
        self.declare_parameter("goal_occupied_radius_m", 0.35)

        self.declare_parameter("cbs_low_level_max_time", 256)
        self.declare_parameter("cbs_max_high_level_nodes", 2000)
        self.declare_parameter("cbs_max_planning_time_sec", 5.0)

        self.reservation_time_step_sec = max(
            0.05,
            float(self.get_parameter("reservation_time_step_sec").value),
        )
        self.nominal_robot_speed_mps = max(
            0.05,
            float(self.get_parameter("nominal_robot_speed_mps").value),
        )
        self.reservation_hold_sec = max(
            0.0,
            float(self.get_parameter("reservation_hold_sec").value),
        )
        self.waypoint_timeout_sec = max(
            1.0,
            float(self.get_parameter("waypoint_timeout_sec").value),
        )
        self.offpath_max_cell_distance = max(
            1,
            int(self.get_parameter("offpath_max_cell_distance").value),
        )
        self.offpath_grace_ticks = max(
            1,
            int(self.get_parameter("offpath_grace_ticks").value),
        )
        self.max_requeue_attempts = max(
            0,
            int(self.get_parameter("max_requeue_attempts").value),
        )
        self.cancel_timeout_sec = max(
            0.1,
            float(self.get_parameter("cancel_timeout_sec").value),
        )
        self.pending_blocked_timeout_sec = max(
            0.0,
            float(self.get_parameter("pending_blocked_timeout_sec").value),
        )
        self.goal_occupied_radius_m = max(
            0.0,
            float(self.get_parameter("goal_occupied_radius_m").value),
        )
        self.cbs_low_level_max_time = max(
            1,
            int(self.get_parameter("cbs_low_level_max_time").value),
        )
        self.cbs_max_high_level_nodes = max(
            1,
            int(self.get_parameter("cbs_max_high_level_nodes").value),
        )
        self.cbs_max_planning_time_sec = max(
            0.0,
            float(self.get_parameter("cbs_max_planning_time_sec").value),
        )

        self.robots: Dict[str, RobotClient] = {
            name: RobotClient(self, name)
            for name in self.robot_names
        }
        self.map_provider = MapProvider(
            self,
            self.map_topic,
            occ_threshold=self.map_occ_threshold,
            unknown_is_free=self.map_unknown_is_free,
        )
        self.cbs_planner = CBSPlanner(self.map_provider.is_free)

        self.pending_tasks: Dict[str, List[TaskRef]] = {
            name: [] for name in self.robot_names
        }
        self.active_goal_for_robot: Dict[str, str] = {}
        self.active_task_id_for_robot: Dict[str, Optional[str]] = {}
        self.task_requeue_attempts: Dict[str, int] = {}
        self.cancel_reason_for_robot: Dict[str, str] = {}
        self.pending_blocked_since_sec: Dict[str, float] = {}
        self.pending_blocked_reason: Dict[str, str] = {}

        self.robot_waypoint_queue: Dict[str, List[WorldPoint]] = {
            name: [] for name in self.robot_names
        }
        self.robot_last_status: Dict[str, str] = {
            name: "unknown" for name in self.robot_names
        }
        self.robot_current_nav_goal_world: Dict[str, Optional[WorldPoint]] = {
            name: None for name in self.robot_names
        }
        self.robot_current_nav_goal_sent_time_sec: Dict[str, Optional[float]] = {
            name: None for name in self.robot_names
        }
        self.robot_active_grid_path: Dict[str, List[GridCell]] = {
            name: [] for name in self.robot_names
        }
        self.robot_offpath_ticks: Dict[str, int] = {
            name: 0 for name in self.robot_names
        }

        self._task_seq = 0

        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_path = share_dir / "config" / "landmarks.yaml"
        try:
            self.landmarks = load_landmarks(str(landmarks_path))
            self.get_logger().info(f"Loaded landmarks from: {landmarks_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load landmarks: {e}")
            self.landmarks = {}

        self.task_sub = self.create_subscription(
            String,
            "/fleet/tasks",
            self._task_callback,
            10,
        )
        self.task_status_pub = self.create_publisher(
            String,
            "/fleet/task_status",
            10,
        )
        self.visualization_pub = self.create_publisher(
            String,
            "/fleet/visualization",
            10,
        )
        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info("FleetManagerNode started")
        self.get_logger().info(f"Robots: {self.robot_names}")
        self.get_logger().info(f"Map topic: {self.map_topic}")
        self.get_logger().info(
            f"Map occupancy: threshold={self.map_occ_threshold}, unknown_is_free={self.map_unknown_is_free}"
        )
        self.get_logger().info(
            "CBS limits: "
            f"low_level_max_time={self.cbs_low_level_max_time}, "
            f"max_high_level_nodes={self.cbs_max_high_level_nodes}, "
            f"max_planning_time_sec={self.cbs_max_planning_time_sec:.2f}s"
        )
        self.get_logger().info("Listening /fleet/tasks")

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _load_robot_names(self) -> List[str]:
        fallback = ["robot1", "robot2"]
        try:
            default_names = self._load_default_robot_names_from_gazebo()
            self.declare_parameter("robot_names", default_names)
            raw_robot_names = self.get_parameter("robot_names").value

            parsed_robot_names: List[str] = []
            if isinstance(raw_robot_names, (list, tuple)):
                for name in raw_robot_names:
                    if isinstance(name, str):
                        clean_name = name.strip().strip("/")
                        if clean_name:
                            parsed_robot_names.append(clean_name)

            if not parsed_robot_names:
                parsed_robot_names = default_names
            names = list(dict.fromkeys(parsed_robot_names))
            return names if names else fallback
        except Exception as e:
            self.get_logger().warn(f"Failed to parse robot_names parameter: {e}")
            return fallback

    def _load_default_robot_names_from_gazebo(self) -> List[str]:
        fallback = ["robot1", "robot2"]
        try:
            gazebo_share = Path(get_package_share_directory("gazebo_sim"))
            robots_path = gazebo_share / "config" / "robots.yaml"
            if not robots_path.exists():
                return fallback

            data = yaml.safe_load(robots_path.read_text(encoding="utf-8")) or {}
            robots_raw = data.get("robots", [])

            parsed: List[str] = []
            if isinstance(robots_raw, list):
                for item in robots_raw:
                    if not isinstance(item, dict):
                        continue
                    name = item.get("name")
                    if isinstance(name, str):
                        clean = name.strip().strip("/")
                        if clean:
                            parsed.append(clean)
            parsed = list(dict.fromkeys(parsed))
            return parsed if parsed else fallback
        except Exception:
            return fallback

    def _now_sec(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _next_task_id(self, robot_name: str) -> str:
        self._task_seq += 1
        return f"{robot_name}-auto-{self._task_seq}"

    def _publish_task_status(
        self,
        robot_name: str,
        goal_name: str,
        state: str,
        reason: str = "",
        requeue_attempts: Optional[int] = None,
        task_id: Optional[str] = None,
    ) -> None:
        payload = {
            "robot": robot_name,
            "goal": goal_name,
            "state": state,
            "reason": reason,
            "requeue_attempts": (
                self.task_requeue_attempts.get(robot_name, 0)
                if requeue_attempts is None
                else requeue_attempts
            ),
        }
        if task_id is not None:
            payload["task_id"] = task_id
        msg = String()
        msg.data = json.dumps(payload)
        self.task_status_pub.publish(msg)
        self.get_logger().info(f"Task status: {msg.data}")

    def _pending_task_key(self, robot_name: str, goal_name: str, task_id: Optional[str]) -> str:
        return task_id or f"{robot_name}:{goal_name}"

    @staticmethod
    def _point_to_payload(point: WorldPoint) -> Dict[str, float]:
        return {"x": float(point[0]), "y": float(point[1])}

    def _publish_visualization_snapshot(self) -> None:
        map_info = self.map_provider.get_grid_info()
        robots_payload: Dict[str, Dict[str, object]] = {}

        for robot_name in self.robot_names:
            pose = self.robots[robot_name].get_pose()
            current_nav_goal = self.robot_current_nav_goal_world.get(robot_name)
            local_path: List[WorldPoint] = []
            if current_nav_goal is not None:
                local_path.append(current_nav_goal)
            local_path.extend(self.robot_waypoint_queue.get(robot_name, []))

            global_path: List[WorldPoint] = []
            if map_info is not None:
                for cell in compress_grid_path(self.robot_active_grid_path.get(robot_name, [])):
                    global_path.append(grid_to_world(cell[0], cell[1], map_info))

            robots_payload[robot_name] = {
                "status": self.robots[robot_name].get_status(),
                "goal": self.active_goal_for_robot.get(robot_name),
                "pose": (
                    None
                    if pose is None
                    else {"x": float(pose.x), "y": float(pose.y)}
                ),
                "current_nav_goal": (
                    None
                    if current_nav_goal is None
                    else self._point_to_payload(current_nav_goal)
                ),
                "local_path": [self._point_to_payload(point) for point in local_path],
                "global_path": [self._point_to_payload(point) for point in global_path],
            }

        msg = String()
        msg.data = json.dumps({"robots": robots_payload})
        self.visualization_pub.publish(msg)

    def _enqueue_pending_task(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
        front: bool = False,
    ) -> int:
        queue = self.pending_tasks.setdefault(robot_name, [])
        item: TaskRef = (goal_name, task_id)
        if front:
            queue.insert(0, item)
        else:
            queue.append(item)
        return len(queue)

    def _peek_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        queue = self.pending_tasks.get(robot_name, [])
        return queue[0] if queue else None

    def _pop_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        queue = self.pending_tasks.get(robot_name, [])
        if not queue:
            return None
        item = queue.pop(0)
        key = self._pending_task_key(robot_name, item[0], item[1])
        self.pending_blocked_since_sec.pop(key, None)
        self.pending_blocked_reason.pop(key, None)
        return item

    def _mark_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
        reason: str,
    ) -> None:
        key = self._pending_task_key(robot_name, goal_name, task_id)
        if key not in self.pending_blocked_since_sec:
            self.pending_blocked_since_sec[key] = self._now_sec()
        last_reason = self.pending_blocked_reason.get(key)
        if last_reason != reason:
            self.pending_blocked_reason[key] = reason
            self._publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="blocked",
                reason=reason,
                task_id=task_id,
            )

    def _clear_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
    ) -> None:
        key = self._pending_task_key(robot_name, goal_name, task_id)
        self.pending_blocked_since_sec.pop(key, None)
        self.pending_blocked_reason.pop(key, None)

    def _expire_blocked_pending_tasks(self) -> None:
        if self.pending_blocked_timeout_sec <= 0.0:
            return

        now_sec = self._now_sec()
        for robot_name in self.robot_names:
            pending_task = self._peek_pending_task(robot_name)
            if pending_task is None:
                continue
            goal_name, task_id = pending_task
            key = self._pending_task_key(robot_name, goal_name, task_id)
            blocked_since = self.pending_blocked_since_sec.get(key)
            if blocked_since is None:
                continue
            elapsed = now_sec - blocked_since
            if elapsed < self.pending_blocked_timeout_sec:
                continue

            popped = self._pop_pending_task(robot_name)
            if popped is None:
                continue

            self._publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="failed",
                reason=f"blocked_timeout_{elapsed:.1f}s",
                task_id=task_id,
            )

    def _has_pending_tasks(self) -> bool:
        return any(self.pending_tasks.get(name, []) for name in self.robot_names)

    def _active_goal_owner(self, goal_name: str, except_robot: str) -> Optional[str]:
        for robot_name, active_goal in self.active_goal_for_robot.items():
            if robot_name == except_robot:
                continue
            if active_goal == goal_name:
                return robot_name
        return None

    def _is_robot_busy(self, status: str) -> bool:
        return status in {"goal_sent", "executing", "canceling"}

    def _task_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Invalid task message: {msg.data}, error={e}")
            return

        robot = data.get("robot")
        if not isinstance(robot, str):
            self.get_logger().error(f"Invalid robot in task message: {msg.data}")
            return
        robot = robot.strip().strip("/")
        if robot not in self.robots:
            self.get_logger().error(f"Unknown robot: {robot}")
            return

        command_raw = data.get("command", "task")
        if not isinstance(command_raw, str):
            self.get_logger().error(f"Invalid command in task message: {msg.data}")
            return
        command = command_raw.strip().lower() or "task"

        task_id = data.get("task_id")
        if task_id is not None and not isinstance(task_id, str):
            self.get_logger().error(f"Invalid task_id type: {type(task_id).__name__}")
            return
        if isinstance(task_id, str):
            task_id = task_id.strip() or None

        if command == "cancel":
            reason = data.get("reason")
            reason_text = reason if isinstance(reason, str) and reason else "user_cancel"
            if self._request_cancel_active_task(robot, reason_text):
                self.get_logger().info(f"[{robot}] cancel requested, reason={reason_text}")
            else:
                self.get_logger().warn(f"[{robot}] cancel requested but no active task")
            return

        goal = data.get("goal")
        if not isinstance(goal, str):
            self.get_logger().error(f"Task message missing valid goal: {msg.data}")
            return
        if goal not in self.landmarks:
            self.get_logger().error(f"Unknown goal: {goal}")
            return

        # preempt is intentionally simplified to normal queued task.
        if command == "preempt":
            self.get_logger().warn(f"[{robot}] preempt disabled in thesis mode, queued as task")
        elif command not in {"task", "goal", "submit"}:
            self.get_logger().error(f"Unknown command '{command}' in task message")
            return

        if task_id is None:
            task_id = self._next_task_id(robot)

        queue_before = len(self.pending_tasks.get(robot, []))
        queue_position = self._enqueue_pending_task(robot, goal, task_id=task_id, front=False)
        if robot not in self.active_goal_for_robot and queue_before == 0:
            self.task_requeue_attempts[robot] = 0

        self._publish_task_status(
            robot_name=robot,
            goal_name=goal,
            state="accepted",
            reason=f"queue_position={queue_position}",
            task_id=task_id,
        )

    def _request_cancel_active_task(self, robot_name: str, reason: str) -> bool:
        goal_name = self.active_goal_for_robot.get(robot_name)
        if goal_name is None:
            return False
        if robot_name in self.cancel_reason_for_robot:
            return True

        if not self.robots[robot_name].cancel_active_goal(timeout_sec=self.cancel_timeout_sec):
            return False

        task_id = self.active_task_id_for_robot.get(robot_name)
        self.cancel_reason_for_robot[robot_name] = reason
        self._publish_task_status(
            robot_name=robot_name,
            goal_name=goal_name,
            state="canceling",
            reason=reason,
            task_id=task_id,
        )
        return True

    def _clear_robot_task_state(self, robot_name: str) -> None:
        self.active_goal_for_robot.pop(robot_name, None)
        self.active_task_id_for_robot.pop(robot_name, None)
        self.cancel_reason_for_robot.pop(robot_name, None)

        self.robot_waypoint_queue[robot_name] = []
        self.robot_current_nav_goal_world[robot_name] = None
        self.robot_current_nav_goal_sent_time_sec[robot_name] = None
        self.robot_active_grid_path[robot_name] = []
        self.robot_offpath_ticks[robot_name] = 0

    def _requeue_active_goal(self, robot_name: str, reason: str) -> None:
        goal_name = self.active_goal_for_robot.get(robot_name)
        if goal_name is None:
            self._clear_robot_task_state(robot_name)
            return
        task_id = self.active_task_id_for_robot.get(robot_name)

        attempts = self.task_requeue_attempts.get(robot_name, 0) + 1
        self.task_requeue_attempts[robot_name] = attempts

        if attempts > self.max_requeue_attempts:
            self._publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="failed",
                reason=reason,
                requeue_attempts=attempts - 1,
                task_id=task_id,
            )
            self.task_requeue_attempts.pop(robot_name, None)
            self._clear_robot_task_state(robot_name)
            return

        self._enqueue_pending_task(robot_name, goal_name, task_id=task_id, front=True)
        self._publish_task_status(
            robot_name=robot_name,
            goal_name=goal_name,
            state="requeued",
            reason=reason,
            requeue_attempts=attempts,
            task_id=task_id,
        )
        self._clear_robot_task_state(robot_name)

    def _on_robot_canceled(self, robot_name: str) -> None:
        goal_name = self.active_goal_for_robot.get(robot_name)
        if goal_name is None:
            self._clear_robot_task_state(robot_name)
            return
        task_id = self.active_task_id_for_robot.get(robot_name)
        reason = self.cancel_reason_for_robot.get(robot_name, "canceled")
        self._publish_task_status(
            robot_name=robot_name,
            goal_name=goal_name,
            state="canceled",
            reason=reason,
            task_id=task_id,
        )
        self.task_requeue_attempts.pop(robot_name, None)
        self._clear_robot_task_state(robot_name)

    def _check_waypoint_timeout(self, robot_name: str, status: str) -> None:
        if robot_name in self.cancel_reason_for_robot:
            return
        if not self._is_robot_busy(status):
            return
        goal_world = self.robot_current_nav_goal_world.get(robot_name)
        if goal_world is None:
            return
        sent_t = self.robot_current_nav_goal_sent_time_sec.get(robot_name)
        if sent_t is None:
            self.robot_current_nav_goal_sent_time_sec[robot_name] = self._now_sec()
            return

        elapsed = self._now_sec() - sent_t
        if elapsed <= self.waypoint_timeout_sec:
            return

        self._requeue_active_goal(robot_name, reason=f"waypoint_timeout_{elapsed:.1f}s")

    def _check_robot_offpath(
        self,
        robot_name: str,
        status: str,
        robot_cell: Optional[GridCell],
    ) -> None:
        if robot_name in self.cancel_reason_for_robot:
            self.robot_offpath_ticks[robot_name] = 0
            return
        if not self._is_robot_busy(status):
            self.robot_offpath_ticks[robot_name] = 0
            return
        if robot_cell is None:
            self.robot_offpath_ticks[robot_name] = 0
            return

        path = self.robot_active_grid_path.get(robot_name, [])
        if not path:
            self.robot_offpath_ticks[robot_name] = 0
            return

        min_dist = min(
            abs(robot_cell[0] - p[0]) + abs(robot_cell[1] - p[1])
            for p in path
        )
        if min_dist <= self.offpath_max_cell_distance:
            self.robot_offpath_ticks[robot_name] = 0
            return

        self.robot_offpath_ticks[robot_name] += 1
        if self.robot_offpath_ticks[robot_name] >= self.offpath_grace_ticks:
            self._requeue_active_goal(
                robot_name,
                reason=f"offpath_dist={min_dist}_ticks={self.robot_offpath_ticks[robot_name]}",
            )

    def _send_next_waypoint(self, robot_name: str) -> bool:
        queue = self.robot_waypoint_queue.get(robot_name, [])
        if not queue:
            return False

        next_x, next_y = queue[0]
        goal_name = self.active_goal_for_robot.get(robot_name)
        yaw = 0.0
        if goal_name is not None and goal_name in self.landmarks:
            yaw = self.landmarks[goal_name].get("yaw", 0.0)

        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        ok = self.robots[robot_name].send_goal(
            x=next_x,
            y=next_y,
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )
        if ok:
            queue.pop(0)
            self.robot_current_nav_goal_world[robot_name] = (next_x, next_y)
            self.robot_current_nav_goal_sent_time_sec[robot_name] = self._now_sec()
            goal_name = self.active_goal_for_robot.get(robot_name)
            if goal_name is not None:
                self._publish_task_status(
                    robot_name=robot_name,
                    goal_name=goal_name,
                    state="executing",
                    reason="waypoint_sent",
                    task_id=self.active_task_id_for_robot.get(robot_name),
                )
        return ok

    def _on_robot_succeeded(self, robot_name: str) -> None:
        self.robot_current_nav_goal_world[robot_name] = None
        self.robot_current_nav_goal_sent_time_sec[robot_name] = None

        if self.robot_waypoint_queue.get(robot_name, []):
            if not self._send_next_waypoint(robot_name):
                self._requeue_active_goal(robot_name, reason="failed_to_send_next_waypoint")
            return

        goal_name = self.active_goal_for_robot.get(robot_name)
        if goal_name is not None:
            self._publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="completed",
                requeue_attempts=self.task_requeue_attempts.get(robot_name, 0),
                task_id=self.active_task_id_for_robot.get(robot_name),
            )
        self.task_requeue_attempts.pop(robot_name, None)
        self._clear_robot_task_state(robot_name)

    def _expand_grid_segment(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        cells: List[GridCell] = [start]
        x, y = start
        gx, gy = goal
        while x != gx:
            x += 1 if gx > x else -1
            cells.append((x, y))
        while y != gy:
            y += 1 if gy > y else -1
            cells.append((x, y))
        return cells

    def _build_active_robot_route_cells(self, robot_name: str, max_steps: int = 80) -> List[GridCell]:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return []
        pose = self.robots[robot_name].get_pose()
        if pose is None:
            return []
        start_cell = world_to_grid(pose.x, pose.y, map_info)
        if start_cell is None:
            return []

        route_cells: List[GridCell] = [start_cell]
        target_waypoints: List[WorldPoint] = []
        current_goal = self.robot_current_nav_goal_world.get(robot_name)
        if current_goal is not None:
            target_waypoints.append(current_goal)
        target_waypoints.extend(self.robot_waypoint_queue.get(robot_name, []))

        for wx, wy in target_waypoints:
            next_cell = world_to_grid(wx, wy, map_info)
            if next_cell is None or next_cell == route_cells[-1]:
                continue
            segment = self._expand_grid_segment(route_cells[-1], next_cell)
            route_cells.extend(segment[1:])
            if len(route_cells) >= max_steps:
                route_cells = route_cells[:max_steps]
                break
        return route_cells

    def _seconds_to_ticks(self, seconds: float) -> int:
        dt = max(self.reservation_time_step_sec, 1e-3)
        return max(1, int(math.ceil(max(0.0, seconds) / dt)))

    def _estimate_transition_ticks(self, from_cell: GridCell, to_cell: GridCell) -> int:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return 1
        manhattan = abs(to_cell[0] - from_cell[0]) + abs(to_cell[1] - from_cell[1])
        if manhattan <= 0:
            return 1
        distance_m = manhattan * map_info.resolution
        speed = max(self.nominal_robot_speed_mps, 0.05)
        return self._seconds_to_ticks(distance_m / speed)

    def _collect_time_reservations_for_active_robots(
        self,
        excluded_robot_names: Set[str],
        hold_horizon: Optional[int] = None,
    ) -> Tuple[List[Tuple[int, GridCell]], List[Tuple[int, GridCell, GridCell]]]:
        reserved_vertices: List[Tuple[int, GridCell]] = []
        reserved_edges: List[Tuple[int, GridCell, GridCell]] = []
        active_statuses = {"goal_sent", "executing", "canceling"}
        hold_ticks = hold_horizon if hold_horizon is not None else self._seconds_to_ticks(
            self.reservation_hold_sec
        )

        for robot_name, robot in self.robots.items():
            if robot_name in excluded_robot_names:
                continue
            if robot.get_status() not in active_statuses:
                continue

            route = self._build_active_robot_route_cells(robot_name)
            if not route:
                continue

            t = 0
            reserved_vertices.append((t, route[0]))
            for idx in range(1, len(route)):
                src = route[idx - 1]
                dst = route[idx]
                move_ticks = self._estimate_transition_ticks(src, dst)
                for local_t in range(move_ticks):
                    edge_t = t + local_t
                    reserved_edges.append((edge_t, src, dst))
                    reserved_edges.append((edge_t, dst, src))
                    if local_t < move_ticks - 1:
                        reserved_vertices.append((edge_t + 1, src))
                t += move_ticks
                reserved_vertices.append((t, dst))

            final_cell = route[-1]
            for dt in range(1, hold_ticks + 1):
                hold_t = t + dt
                reserved_vertices.append((hold_t, final_cell))
                reserved_edges.append((hold_t - 1, final_cell, final_cell))

        return reserved_vertices, reserved_edges

    def _collect_current_robot_cells(
        self,
        map_info,
    ) -> Dict[str, GridCell]:
        occupied: Dict[str, GridCell] = {}
        for robot_name, robot in self.robots.items():
            pose = robot.get_pose()
            if pose is None:
                continue
            cell = world_to_grid(pose.x, pose.y, map_info)
            if cell is None:
                continue
            occupied[robot_name] = cell
        return occupied

    def _find_robot_occupying_goal(
        self,
        robot_name: str,
        goal_name: str,
        goal_cell: GridCell,
        occupied_cells: Dict[str, GridCell],
    ) -> Optional[str]:
        goal_lm = self.landmarks.get(goal_name)
        goal_x = None if goal_lm is None else float(goal_lm["x"])
        goal_y = None if goal_lm is None else float(goal_lm["y"])
        radius_sq = self.goal_occupied_radius_m * self.goal_occupied_radius_m

        for other_robot, other_cell in occupied_cells.items():
            if other_robot == robot_name:
                continue
            if other_cell == goal_cell:
                return other_robot

            if goal_x is None or goal_y is None or radius_sq <= 0.0:
                continue

            other_pose = self.robots[other_robot].get_pose()
            if other_pose is None:
                continue
            dx = other_pose.x - goal_x
            dy = other_pose.y - goal_y
            if dx * dx + dy * dy <= radius_sq:
                return other_robot

        return None

    def _collect_static_blocked_cells(
        self,
        excluded_robot_names: Set[str],
    ) -> List[GridCell]:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return []

        active_statuses = {"goal_sent", "executing", "canceling"}
        blocked: List[GridCell] = []
        for robot_name, robot in self.robots.items():
            if robot_name in excluded_robot_names:
                continue
            # Any robot that is not currently moving is treated as a static obstacle
            # for the next planning step, including idle and succeeded robots.
            if robot.get_status() in active_statuses:
                continue
            pose = robot.get_pose()
            if pose is None:
                continue
            cell = world_to_grid(pose.x, pose.y, map_info)
            if cell is not None:
                blocked.append(cell)
        return blocked

    def _build_robot_requests(self) -> Tuple[List[RobotRequest], Dict[str, TaskRef]]:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return [], {}

        requests: List[RobotRequest] = []
        goal_info_by_robot: Dict[str, TaskRef] = {}
        used_goal_cells: Dict[GridCell, str] = {}
        occupied_cells = self._collect_current_robot_cells(map_info)

        for robot_name in self.robot_names:
            pending_task = self._peek_pending_task(robot_name)
            if pending_task is None:
                continue
            goal_name, task_id = pending_task

            active_owner = self._active_goal_owner(goal_name, except_robot=robot_name)
            if active_owner is not None:
                self._mark_pending_blocked(
                    robot_name=robot_name,
                    goal_name=goal_name,
                    task_id=task_id,
                    reason=f"goal_active_on_{active_owner}",
                )
                continue

            pose = self.robots[robot_name].get_pose()
            if pose is None:
                continue
            lm = self.landmarks[goal_name]

            start_cell = world_to_grid(pose.x, pose.y, map_info)
            goal_cell = world_to_grid(lm["x"], lm["y"], map_info)
            if start_cell is None or goal_cell is None:
                continue

            if not self.map_provider.is_free(goal_cell[0], goal_cell[1]):
                adjusted_goal = find_nearest_free_cell(goal_cell, self.map_provider.is_free, max_radius=10)
                if adjusted_goal is None:
                    continue
                goal_cell = adjusted_goal

            occupied_by_robot = self._find_robot_occupying_goal(
                robot_name=robot_name,
                goal_name=goal_name,
                goal_cell=goal_cell,
                occupied_cells=occupied_cells,
            )
            if occupied_by_robot is not None:
                self._mark_pending_blocked(
                    robot_name=robot_name,
                    goal_name=goal_name,
                    task_id=task_id,
                    reason=f"goal_occupied_by_{occupied_by_robot}",
                )
                self.get_logger().info(
                    f"[{robot_name}] waiting: goal {goal_name} is currently occupied by {occupied_by_robot}"
                )
                continue

            if goal_cell in used_goal_cells:
                self._mark_pending_blocked(
                    robot_name=robot_name,
                    goal_name=goal_name,
                    task_id=task_id,
                    reason=f"goal_reserved_in_batch_by_{used_goal_cells[goal_cell]}",
                )
                continue
            self._clear_pending_blocked(robot_name, goal_name, task_id)
            used_goal_cells[goal_cell] = robot_name

            requests.append(
                RobotRequest(
                    robot_name=robot_name,
                    start=start_cell,
                    goal=goal_cell,
                )
            )
            goal_info_by_robot[robot_name] = (goal_name, task_id)

        return requests, goal_info_by_robot

    def _apply_cbs_result(
        self,
        robot_requests: List[RobotRequest],
        goal_info_by_robot: Dict[str, TaskRef],
        blocked_cells: Optional[List[GridCell]] = None,
        reserved_vertices: Optional[List[Tuple[int, GridCell]]] = None,
        reserved_edges: Optional[List[Tuple[int, GridCell, GridCell]]] = None,
    ) -> None:
        planner_result = self.cbs_planner.plan_for_robots(
            robot_requests,
            blocked_cells=blocked_cells,
            reserved_vertex_constraints=reserved_vertices,
            reserved_edge_constraints=reserved_edges,
            low_level_max_time=self.cbs_low_level_max_time,
            max_high_level_nodes=self.cbs_max_high_level_nodes,
            max_planning_time_sec=self.cbs_max_planning_time_sec,
        )

        if not planner_result.plans:
            reason = planner_result.debug.reason
            for req in robot_requests:
                goal_name, task_id = goal_info_by_robot.get(req.robot_name, ("", None))
                self._publish_task_status(
                    robot_name=req.robot_name,
                    goal_name=goal_name,
                    state="planning_failed",
                    reason=reason,
                    task_id=task_id,
                )
            return

        map_info = self.map_provider.get_grid_info()
        assert map_info is not None

        planned_robots: List[str] = []
        for req in robot_requests:
            robot_name = req.robot_name
            plan = planner_result.plans.get(robot_name)
            if plan is None:
                continue
            robot_pose = self.robots[robot_name].get_pose()
            if robot_pose is None:
                continue

            compressed_path = compress_grid_path(plan.path)
            world_waypoints: List[WorldPoint] = []
            for i, j in compressed_path:
                wx, wy = grid_to_world(i, j, map_info)
                world_waypoints.append((wx, wy))
            if not world_waypoints:
                continue

            filtered_waypoints: List[WorldPoint] = []
            for wx, wy in world_waypoints:
                dist = ((wx - robot_pose.x) ** 2 + (wy - robot_pose.y) ** 2) ** 0.5
                if dist > 0.10:
                    filtered_waypoints.append((wx, wy))
            if not filtered_waypoints:
                filtered_waypoints = [world_waypoints[-1]]

            goal_name, task_id = goal_info_by_robot[robot_name]
            self.active_goal_for_robot[robot_name] = goal_name
            self.active_task_id_for_robot[robot_name] = task_id
            self.task_requeue_attempts.setdefault(robot_name, 0)
            self.robot_offpath_ticks[robot_name] = 0
            self.robot_active_grid_path[robot_name] = plan.path[:]
            self.robot_waypoint_queue[robot_name] = filtered_waypoints
            self._publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="planned",
                task_id=task_id,
            )
            planned_robots.append(robot_name)

        started_robots: List[str] = []
        for robot_name in planned_robots:
            if self._send_next_waypoint(robot_name):
                started_robots.append(robot_name)
            else:
                self._requeue_active_goal(robot_name, reason="failed_to_send_first_waypoint")

        for robot_name in started_robots:
            popped = self._pop_pending_task(robot_name)
            expected = goal_info_by_robot.get(robot_name)
            if popped is None or expected is None:
                continue
            if popped != expected:
                self.get_logger().warn(
                    f"[{robot_name}] pending mismatch: popped={popped}, expected={expected}"
                )

    def _try_plan_pending_tasks_with_cbs(self) -> None:
        if not self._has_pending_tasks():
            return

        robot_requests, goal_info_by_robot = self._build_robot_requests()
        if not robot_requests:
            return

        ready_requests: List[RobotRequest] = []
        for req in robot_requests:
            status = self.robots[req.robot_name].get_status()
            if not self._is_robot_busy(status):
                ready_requests.append(req)

        if not ready_requests:
            return

        ready_names = {req.robot_name for req in ready_requests}
        blocked_cells = self._collect_static_blocked_cells(excluded_robot_names=ready_names)
        reserved_vertices, reserved_edges = self._collect_time_reservations_for_active_robots(
            excluded_robot_names=ready_names
        )

        self._apply_cbs_result(
            ready_requests,
            goal_info_by_robot=goal_info_by_robot,
            blocked_cells=blocked_cells,
            reserved_vertices=reserved_vertices,
            reserved_edges=reserved_edges,
        )

    def _tick(self) -> None:
        if not self.map_provider.has_map():
            self.get_logger().info(f"Waiting for {self.map_topic} ...")
            self._publish_visualization_snapshot()
            return

        map_info = self.map_provider.get_grid_info()
        assert map_info is not None

        for name, robot in self.robots.items():
            pose = robot.get_pose()
            status = robot.get_status()
            prev_status = self.robot_last_status.get(name, "unknown")

            if pose is None:
                self.robot_last_status[name] = status
                continue

            robot_cell = world_to_grid(pose.x, pose.y, map_info)
            self._check_waypoint_timeout(name, status)
            self._check_robot_offpath(name, status, robot_cell)

            if prev_status != status:
                if status == "succeeded":
                    self._on_robot_succeeded(name)
                elif status == "canceled":
                    if name in self.cancel_reason_for_robot:
                        self._on_robot_canceled(name)
                    else:
                        self._requeue_active_goal(name, reason="canceled")
                elif status in {"rejected", "aborted", "server_unavailable", "response_error", "result_error"}:
                    self._requeue_active_goal(name, reason=status)
                elif status.startswith("finished_"):
                    self._requeue_active_goal(name, reason=status)
            elif status == "succeeded" and name in self.active_goal_for_robot:
                # Recovery path if transition to succeeded was already observed earlier.
                self._on_robot_succeeded(name)

            self.robot_last_status[name] = status

        self._expire_blocked_pending_tasks()
        self._try_plan_pending_tasks_with_cbs()
        self._publish_visualization_snapshot()


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
