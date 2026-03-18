from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from fleet_msgs.msg import FleetTask, FleetTaskStatus, FleetVisualization, RobotVisualization, Point2D

from scripts.planning.cbs_planner import CBSPlanner, RobotRequest
from scripts.planning.search_utils import find_nearest_free_cell
from scripts.runtime.map_provider import MapProvider
from scripts.runtime.robot_client import RobotClient
from scripts.utils.geometry_utils import yaw_to_quaternion
from scripts.utils.grid_utils import world_to_grid
from scripts.utils.landmark_loader import load_landmarks
from scripts.execution_state import ExecutionState
from scripts.landmark_router import LandmarkRouter
from scripts.task_state import TaskStateStore
from scripts.visualization_snapshot import VisualizationSnapshotBuilder

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
        self.declare_parameter("waypoint_min_spacing_m", 0.30)
        self.declare_parameter("offpath_max_cell_distance", 2)
        self.declare_parameter("offpath_grace_ticks", 5)
        self.declare_parameter("max_requeue_attempts", 5)
        self.declare_parameter("cancel_timeout_sec", 2.0)
        self.declare_parameter("pending_blocked_timeout_sec", 120.0)
        self.declare_parameter("goal_occupied_radius_m", 0.35)
        self.declare_parameter("occupied_landmark_radius_m", 0.9)
        self.declare_parameter("occupied_landmark_block_radius_m", 0.9)
        self.declare_parameter("occupied_landmark_block_radius_cells", 1)
        self.declare_parameter("landmark_capture_radius_m", 0.35)

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
        self.waypoint_min_spacing_m = max(
            0.05,
            float(self.get_parameter("waypoint_min_spacing_m").value),
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
        self.occupied_landmark_radius_m = max(
            0.0,
            float(self.get_parameter("occupied_landmark_radius_m").value),
        )
        self.occupied_landmark_block_radius_m = max(
            0.0,
            float(self.get_parameter("occupied_landmark_block_radius_m").value),
        )
        self.occupied_landmark_block_radius_cells = max(
            0,
            int(self.get_parameter("occupied_landmark_block_radius_cells").value),
        )
        self.landmark_capture_radius_m = max(
            0.05,
            float(self.get_parameter("landmark_capture_radius_m").value),
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

        self.task_state = TaskStateStore(self.robot_names)
        self.pending_tasks = self.task_state.pending_tasks
        self.active_goal_for_robot = self.task_state.active_goal_for_robot
        self.active_task_id_for_robot = self.task_state.active_task_id_for_robot
        self.task_requeue_attempts = self.task_state.task_requeue_attempts
        self.cancel_reason_for_robot = self.task_state.cancel_reason_for_robot
        self.pending_blocked_since_sec = self.task_state.pending_blocked_since_sec
        self.pending_blocked_reason = self.task_state.pending_blocked_reason

        self.execution_state = ExecutionState(self.robot_names)
        self.robot_waypoint_queue = self.execution_state.robot_waypoint_queue
        self.robot_waypoint_landmark_queue = self.execution_state.robot_waypoint_landmark_queue
        self.robot_planned_landmark_path = self.execution_state.robot_planned_landmark_path
        self.robot_last_status: Dict[str, str] = {
            name: "unknown" for name in self.robot_names
        }
        self.robot_current_landmark: Dict[str, Optional[str]] = {
            name: None for name in self.robot_names
        }
        self.robot_current_nav_goal_world = self.execution_state.robot_current_nav_goal_world
        self.robot_current_nav_goal_sent_time_sec = self.execution_state.robot_current_nav_goal_sent_time_sec
        self.robot_active_grid_path = self.execution_state.robot_active_grid_path
        self.robot_offpath_ticks = self.execution_state.robot_offpath_ticks

        self._task_seq = 0

        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_path = share_dir / "config" / "landmarks.yaml"
        try:
            self.landmarks = load_landmarks(str(landmarks_path))
            self.get_logger().info(f"Loaded landmarks from: {landmarks_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load landmarks: {e}")
            self.landmarks = {}
        self.landmark_router = LandmarkRouter(
            self.landmarks,
            default_capture_radius_m=self.landmark_capture_radius_m,
        )
        self.visualization_builder = VisualizationSnapshotBuilder(self.landmarks)

        self.task_sub = self.create_subscription(
            FleetTask,
            "/fleet/tasks",
            self._task_callback,
            10,
        )
        self.task_status_pub = self.create_publisher(
            FleetTaskStatus,
            "/fleet/task_status",
            10,
        )
        self.visualization_pub = self.create_publisher(
            FleetVisualization,
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
        msg = FleetTaskStatus()
        msg.robot = robot_name
        msg.goal = goal_name
        msg.state = state
        msg.reason = reason
        msg.requeue_attempts = (
            self.task_requeue_attempts.get(robot_name, 0)
            if requeue_attempts is None
            else requeue_attempts
        )
        msg.task_id = task_id or ""
        self.task_status_pub.publish(msg)
        self.get_logger().info(
            "Task status: "
            f"robot={msg.robot}, goal={msg.goal}, state={msg.state}, "
            f"reason={msg.reason}, requeue_attempts={msg.requeue_attempts}, task_id={msg.task_id}"
        )

    def _pending_task_key(self, robot_name: str, goal_name: str, task_id: Optional[str]) -> str:
        return self._get_task_state().pending_task_key(robot_name, goal_name, task_id)

    def _get_landmark_router(self) -> LandmarkRouter:
        router = getattr(self, "landmark_router", None)
        if router is None or router.landmarks is not self.landmarks:
            capture_radius_m = getattr(self, "landmark_capture_radius_m", 0.35)
            router = LandmarkRouter(
                self.landmarks,
                default_capture_radius_m=capture_radius_m,
            )
            self.landmark_router = router
        return router

    def _get_visualization_builder(self) -> VisualizationSnapshotBuilder:
        builder = getattr(self, "visualization_builder", None)
        if builder is None or builder.landmarks is not self.landmarks:
            builder = VisualizationSnapshotBuilder(self.landmarks)
            self.visualization_builder = builder
        return builder

    def _get_task_state(self) -> TaskStateStore:
        state = getattr(self, "task_state", None)
        if state is None:
            state = TaskStateStore(getattr(self, "robot_names", []))
            state.pending_tasks = getattr(self, "pending_tasks", state.pending_tasks)
            state.active_goal_for_robot = getattr(self, "active_goal_for_robot", state.active_goal_for_robot)
            state.active_task_id_for_robot = getattr(
                self,
                "active_task_id_for_robot",
                state.active_task_id_for_robot,
            )
            state.task_requeue_attempts = getattr(
                self,
                "task_requeue_attempts",
                state.task_requeue_attempts,
            )
            state.cancel_reason_for_robot = getattr(
                self,
                "cancel_reason_for_robot",
                state.cancel_reason_for_robot,
            )
            state.pending_blocked_since_sec = getattr(
                self,
                "pending_blocked_since_sec",
                state.pending_blocked_since_sec,
            )
            state.pending_blocked_reason = getattr(
                self,
                "pending_blocked_reason",
                state.pending_blocked_reason,
            )
            self.task_state = state
        return state

    def _get_execution_state(self) -> ExecutionState:
        state = getattr(self, "execution_state", None)
        if state is None:
            state = ExecutionState(getattr(self, "robot_names", []))
            state.robot_waypoint_queue = getattr(self, "robot_waypoint_queue", state.robot_waypoint_queue)
            state.robot_waypoint_landmark_queue = getattr(
                self,
                "robot_waypoint_landmark_queue",
                state.robot_waypoint_landmark_queue,
            )
            state.robot_planned_landmark_path = getattr(
                self,
                "robot_planned_landmark_path",
                state.robot_planned_landmark_path,
            )
            state.robot_current_nav_goal_world = getattr(
                self,
                "robot_current_nav_goal_world",
                state.robot_current_nav_goal_world,
            )
            state.robot_current_nav_goal_sent_time_sec = getattr(
                self,
                "robot_current_nav_goal_sent_time_sec",
                state.robot_current_nav_goal_sent_time_sec,
            )
            state.robot_active_grid_path = getattr(
                self,
                "robot_active_grid_path",
                state.robot_active_grid_path,
            )
            state.robot_offpath_ticks = getattr(self, "robot_offpath_ticks", state.robot_offpath_ticks)
            self.execution_state = state
        return state

    def _publish_visualization_snapshot(self) -> None:
        robots_payload = self._get_visualization_builder().build(
            robot_names=self.robot_names,
            robots=self.robots,
            active_goal_for_robot=self.active_goal_for_robot,
            current_nav_goals=self.robot_current_nav_goal_world,
            waypoint_landmark_queues=self.robot_waypoint_landmark_queue,
            planned_landmark_paths=self.robot_planned_landmark_path,
        )

        msg = FleetVisualization()
        for robot_name in self.robot_names:
            robot_data = robots_payload.get(robot_name, {})
            robot_msg = RobotVisualization()
            robot_msg.robot = robot_name
            robot_msg.status = str(robot_data.get("status", ""))
            robot_msg.goal = str(robot_data.get("goal", ""))
            pose = robot_data.get("pose")
            if isinstance(pose, dict):
                robot_msg.has_pose = True
                robot_msg.pose = Point2D()
                robot_msg.pose.x = float(pose.get("x", 0.0))
                robot_msg.pose.y = float(pose.get("y", 0.0))
            current_nav_goal = robot_data.get("current_nav_goal")
            if isinstance(current_nav_goal, dict):
                robot_msg.has_current_nav_goal = True
                robot_msg.current_nav_goal = Point2D()
                robot_msg.current_nav_goal.x = float(current_nav_goal.get("x", 0.0))
                robot_msg.current_nav_goal.y = float(current_nav_goal.get("y", 0.0))
            for path_name in ("local_path", "global_path"):
                point_payloads = robot_data.get(path_name, [])
                msg_points = []
                if isinstance(point_payloads, list):
                    for point in point_payloads:
                        if not isinstance(point, dict):
                            continue
                        point_msg = Point2D()
                        point_msg.x = float(point.get("x", 0.0))
                        point_msg.y = float(point.get("y", 0.0))
                        msg_points.append(point_msg)
                setattr(robot_msg, path_name, msg_points)
            msg.robots.append(robot_msg)
        self.visualization_pub.publish(msg)

    def _enqueue_pending_task(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
        front: bool = False,
    ) -> int:
        return self._get_task_state().enqueue_pending_task(robot_name, goal_name, task_id, front=front)

    def _peek_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        return self._get_task_state().peek_pending_task(robot_name)

    def _pop_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        return self._get_task_state().pop_pending_task(robot_name)

    def _mark_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
        reason: str,
    ) -> None:
        self._get_task_state().mark_pending_blocked(
            robot_name=robot_name,
            goal_name=goal_name,
            task_id=task_id,
            reason=reason,
            now_sec=self._now_sec(),
            publish_task_status=self._publish_task_status,
        )

    def _clear_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
    ) -> None:
        self._get_task_state().clear_pending_blocked(robot_name, goal_name, task_id)

    def _expire_blocked_pending_tasks(self) -> None:
        self._get_task_state().expire_blocked_pending_tasks(
            robot_names=self.robot_names,
            now_sec=self._now_sec(),
            pending_blocked_timeout_sec=self.pending_blocked_timeout_sec,
            publish_task_status=self._publish_task_status,
        )

    def _has_pending_tasks(self) -> bool:
        return self._get_task_state().has_pending_tasks(self.robot_names)

    def _active_goal_owner(self, goal_name: str, except_robot: str) -> Optional[str]:
        return self._get_task_state().active_goal_owner(goal_name, except_robot)

    def _is_robot_busy(self, status: str) -> bool:
        return status in {"goal_sent", "executing", "canceling"}

    def _task_callback(self, msg: FleetTask) -> None:
        robot = msg.robot
        if not isinstance(robot, str):
            self.get_logger().error("Invalid robot in task message")
            return
        robot = robot.strip().strip("/")
        if robot not in self.robots:
            self.get_logger().error(f"Unknown robot: {robot}")
            return

        command_raw = msg.command or "task"
        if not isinstance(command_raw, str):
            self.get_logger().error("Invalid command in task message")
            return
        command = command_raw.strip().lower() or "task"

        task_id = msg.task_id
        if task_id is not None and not isinstance(task_id, str):
            self.get_logger().error(f"Invalid task_id type: {type(task_id).__name__}")
            return
        if isinstance(task_id, str):
            task_id = task_id.strip() or None

        if command == "cancel":
            reason_text = msg.reason if isinstance(msg.reason, str) and msg.reason else "user_cancel"
            if self._request_cancel_active_task(robot, reason_text):
                self.get_logger().info(f"[{robot}] cancel requested, reason={reason_text}")
            else:
                self.get_logger().warn(f"[{robot}] cancel requested but no active task")
            return

        goal = msg.goal
        if not isinstance(goal, str):
            self.get_logger().error("Task message missing valid goal")
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
        self._get_execution_state().clear_robot(robot_name)

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
        active_goal = self.active_goal_for_robot.get(robot_name)
        if active_goal is not None and self.robot_current_landmark.get(robot_name) == active_goal:
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
        landmark_queue = self.robot_waypoint_landmark_queue.get(robot_name, [])
        next_landmark = landmark_queue[0] if landmark_queue else None
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
            if landmark_queue:
                landmark_queue.pop(0)
            self.robot_current_nav_goal_world[robot_name] = (next_x, next_y)
            self.robot_current_nav_goal_sent_time_sec[robot_name] = self._now_sec()
            self.get_logger().info(
                f"[{robot_name}] nav2_next_lm={next_landmark} "
                f"world=({next_x:.3f}, {next_y:.3f}) "
                f"remaining_lm_queue={landmark_queue}"
            )
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

    def _inflate_blocked_cells(self, center: GridCell, radius_cells: int) -> List[GridCell]:
        if radius_cells <= 0:
            return [center]

        cx, cy = center
        cells: List[GridCell] = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                if abs(dx) + abs(dy) > radius_cells:
                    continue
                cells.append((cx + dx, cy + dy))
        return cells

    def _inflate_blocked_cells_meters(
        self,
        center: GridCell,
        map_info,
        radius_m: float,
        fallback_radius_cells: int = 0,
    ) -> List[GridCell]:
        if radius_m > 0.0 and map_info.resolution > 0.0:
            radius_cells = max(0, int(math.ceil(radius_m / map_info.resolution)))
        else:
            radius_cells = max(0, int(fallback_radius_cells))
        return self._inflate_blocked_cells(center, radius_cells)

    def _format_blocked_cells_for_log(self, blocked_cells: List[GridCell], map_info) -> str:
        return self._get_landmark_router().format_blocked_cells_for_log(blocked_cells, map_info)

    def _sparsify_world_waypoints(
        self,
        waypoints: List[WorldPoint],
        min_spacing_m: float,
    ) -> List[WorldPoint]:
        if len(waypoints) <= 2:
            return waypoints[:]

        kept: List[WorldPoint] = [waypoints[0]]
        last_x, last_y = waypoints[0]
        threshold_sq = max(min_spacing_m, 0.0) ** 2

        for wx, wy in waypoints[1:-1]:
            dx = wx - last_x
            dy = wy - last_y
            if dx * dx + dy * dy >= threshold_sq:
                kept.append((wx, wy))
                last_x, last_y = wx, wy

        if kept[-1] != waypoints[-1]:
            kept.append(waypoints[-1])

        return kept

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
        blocked: Set[GridCell] = set()
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
                blocked.update(
                    self._inflate_blocked_cells_meters(
                        cell,
                        map_info,
                        self.occupied_landmark_block_radius_m,
                        self.occupied_landmark_block_radius_cells,
                    )
                )

            lm_name = self._get_landmark_router().nearest_landmark_name(
                pose.x,
                pose.y,
                max_distance=self.occupied_landmark_radius_m,
            )
            if lm_name is None:
                continue

            lm = self.landmarks.get(lm_name)
            if lm is None:
                continue

            lm_cell = world_to_grid(float(lm["x"]), float(lm["y"]), map_info)
            if lm_cell is not None:
                blocked.update(
                    self._inflate_blocked_cells_meters(
                        lm_cell,
                        map_info,
                        self.occupied_landmark_block_radius_m,
                        self.occupied_landmark_block_radius_cells,
                    )
                )
        return sorted(blocked)

    def _build_robot_requests(self) -> Tuple[List[RobotRequest], Dict[str, TaskRef]]:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return [], {}

        robot_current_landmark = getattr(self, "robot_current_landmark", {})
        requests: List[RobotRequest] = []
        goal_info_by_robot: Dict[str, TaskRef] = {}
        used_goal_cells: Dict[GridCell, str] = {}
        occupied_cells = self._collect_current_robot_cells(map_info)

        for robot_name in self.robot_names:
            pending_task = self._peek_pending_task(robot_name)
            if pending_task is None:
                continue
            goal_name, task_id = pending_task
            if robot_current_landmark.get(robot_name) == goal_name:
                popped = self._pop_pending_task(robot_name)
                if popped is not None:
                    self._clear_pending_blocked(robot_name, goal_name, task_id)
                    self.task_requeue_attempts.pop(robot_name, None)
                    self._publish_task_status(
                        robot_name=robot_name,
                        goal_name=goal_name,
                        state="completed",
                        reason="already_at_goal_landmark",
                        task_id=task_id,
                    )
                continue

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

            landmark_sequence = self._get_landmark_router().landmark_sequence_from_grid_path(
                plan.path,
                map_info,
                capture_radius_m=self.landmark_capture_radius_m,
            )
            goal_name, task_id = goal_info_by_robot[robot_name]
            if not landmark_sequence or landmark_sequence[-1] != goal_name:
                landmark_sequence.append(goal_name)

            start_landmark = self.robot_current_landmark.get(robot_name)
            occupied_landmarks = {
                other_lm
                for other_robot, other_lm in self.robot_current_landmark.items()
                if other_robot != robot_name and other_lm is not None
            }
            if start_landmark is not None:
                graph_landmark_sequence = self._get_landmark_router().plan_route(
                    start_landmark,
                    goal_name,
                    blocked_landmarks=occupied_landmarks,
                )
                if graph_landmark_sequence is not None:
                    landmark_sequence = graph_landmark_sequence

            if start_landmark is not None and landmark_sequence and landmark_sequence[0] == start_landmark:
                landmark_sequence = landmark_sequence[1:]

            conflicting_landmarks = [
                lm_name for lm_name in landmark_sequence[:-1]
                if lm_name in occupied_landmarks
            ]
            if conflicting_landmarks:
                self.get_logger().warn(
                    f"[{robot_name}] rejecting plan through occupied landmarks: {conflicting_landmarks}"
                )
                self._publish_task_status(
                    robot_name=robot_name,
                    goal_name=goal_name,
                    state="planning_failed",
                    reason=f"occupied_landmark_in_path:{','.join(conflicting_landmarks)}",
                    task_id=task_id,
                )
                continue

            world_waypoints: List[WorldPoint] = []
            waypoint_landmarks: List[str] = []
            for lm_name in landmark_sequence:
                lm = self.landmarks.get(lm_name)
                if lm is None:
                    continue
                world_waypoints.append((float(lm["x"]), float(lm["y"])))
                waypoint_landmarks.append(lm_name)
            if not world_waypoints:
                continue

            filtered_waypoints: List[WorldPoint] = []
            filtered_landmarks: List[str] = []
            for (wx, wy), lm_name in zip(world_waypoints, waypoint_landmarks):
                dist = ((wx - robot_pose.x) ** 2 + (wy - robot_pose.y) ** 2) ** 0.5
                if dist > 0.10:
                    filtered_waypoints.append((wx, wy))
                    filtered_landmarks.append(lm_name)
            if not filtered_waypoints:
                filtered_waypoints = [world_waypoints[-1]]
                filtered_landmarks = [waypoint_landmarks[-1]]

            self.active_goal_for_robot[robot_name] = goal_name
            self.active_task_id_for_robot[robot_name] = task_id
            self.task_requeue_attempts.setdefault(robot_name, 0)
            self.robot_offpath_ticks[robot_name] = 0
            self.robot_active_grid_path[robot_name] = plan.path[:]
            self.robot_waypoint_queue[robot_name] = filtered_waypoints
            self.robot_waypoint_landmark_queue[robot_name] = filtered_landmarks[:]
            self.robot_planned_landmark_path[robot_name] = landmark_sequence[:]
            self.get_logger().info(
                f"[{robot_name}] cbs_lm_plan={landmark_sequence} exec_lm_queue={filtered_landmarks}"
            )
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
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
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
        occupied_landmarks = {
            robot_name: lm_name
            for robot_name, lm_name in self.robot_current_landmark.items()
            if lm_name is not None
        }
        self.get_logger().info(
            "CBS planning snapshot: "
            f"requests={[(req.robot_name, req.start, req.goal) for req in ready_requests]}, "
            f"occupied_landmarks={occupied_landmarks}, "
            f"blocked_cells={self._format_blocked_cells_for_log(blocked_cells, map_info)}"
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
            prev_landmark = self.robot_current_landmark.get(name)
            if pose is not None:
                current_landmark = self._get_landmark_router().nearest_landmark_name(
                    pose.x,
                    pose.y,
                    max_distance=self.occupied_landmark_radius_m,
                )
                self.robot_current_landmark[name] = current_landmark
                if current_landmark != prev_landmark:
                    self.get_logger().info(
                        f"[{name}] current_landmark: {prev_landmark} -> {current_landmark}"
                    )
            else:
                self.robot_current_landmark[name] = None

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
