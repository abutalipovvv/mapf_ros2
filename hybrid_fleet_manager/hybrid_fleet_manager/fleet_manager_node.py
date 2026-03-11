from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, List, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from hybrid_fleet_manager.cbs_planner import CBSPlanner, RobotRequest
from hybrid_fleet_manager.geometry_utils import yaw_to_quaternion
from hybrid_fleet_manager.grid_utils import grid_to_world, world_to_grid
from hybrid_fleet_manager.landmark_loader import load_landmarks
from hybrid_fleet_manager.map_provider import MapProvider
from hybrid_fleet_manager.path_utils import compress_grid_path
from hybrid_fleet_manager.robot_client import RobotClient
from hybrid_fleet_manager.search_utils import find_nearest_free_cell


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__("fleet_manager_node")

        self.robot_names = ["robot1", "robot2"]

        self.robots: Dict[str, RobotClient] = {
            name: RobotClient(self, name)
            for name in self.robot_names
        }

        self.map_provider = MapProvider(self, "/robot1/map")
        self.cbs_planner = CBSPlanner(self.map_provider.is_free)

        self.robot_waypoint_queue: Dict[str, List[Tuple[float, float]]] = {
            name: [] for name in self.robot_names
        }
        self.robot_last_status: Dict[str, str] = {
            name: "unknown" for name in self.robot_names
        }

        share_dir = Path(get_package_share_directory("hybrid_fleet_manager"))
        landmarks_path = share_dir / "config" / "landmarks.yaml"

        try:
            self.landmarks = load_landmarks(str(landmarks_path))
            self.get_logger().info(f"Loaded landmarks from: {landmarks_path}")
            self.get_logger().info(f"Available landmarks: {list(self.landmarks.keys())}")
        except Exception as e:
            self.get_logger().error(f"Failed to load landmarks: {e}")
            self.landmarks = {}

        self.pending_tasks: Dict[str, str] = {}
        self.active_goal_for_robot: Dict[str, str] = {}

        self.task_sub = self.create_subscription(
            String,
            "/fleet/tasks",
            self._task_callback,
            10,
        )

        self.timer = self.create_timer(1.0, self._tick)

        self.get_logger().info("FleetManagerNode started")
        self.get_logger().info("Listening /fleet/tasks")

    def _task_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            robot = data["robot"]
            goal = data["goal"]
        except Exception as e:
            self.get_logger().error(f"Invalid task message: {msg.data}, error={e}")
            return

        if robot not in self.robots:
            self.get_logger().error(f"Unknown robot: {robot}")
            return

        if goal not in self.landmarks:
            self.get_logger().error(f"Unknown goal: {goal}")
            return

        self.pending_tasks[robot] = goal
        self.get_logger().info(f"Task received: {robot} -> {goal}")

    def _send_next_waypoint(self, robot_name: str) -> bool:
        if robot_name not in self.robot_waypoint_queue:
            self.get_logger().warn(f"[{robot_name}] no waypoint queue")
            return False

        queue = self.robot_waypoint_queue[robot_name]
        if not queue:
            self.get_logger().info(f"[{robot_name}] waypoint queue is empty")
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
            self.get_logger().info(
                f"[{robot_name}] sent next waypoint: ({next_x:.2f}, {next_y:.2f}), "
                f"remaining={len(queue)}"
            )
        else:
            self.get_logger().error(f"[{robot_name}] failed to send next waypoint")

        return ok

    def _clear_robot_task_state(self, robot_name: str):
        if robot_name in self.active_goal_for_robot:
            del self.active_goal_for_robot[robot_name]

        self.robot_waypoint_queue[robot_name] = []
        self.get_logger().info(f"[{robot_name}] task state cleared")

    def _on_robot_succeeded(self, robot_name: str):
        queue = self.robot_waypoint_queue.get(robot_name, [])

        if queue:
            self.get_logger().info(
                f"[{robot_name}] previous waypoint succeeded, sending next one"
            )
            self._send_next_waypoint(robot_name)
        else:
            self.get_logger().info(
                f"[{robot_name}] final waypoint reached, task complete"
            )
            self._clear_robot_task_state(robot_name)

    def _build_robot_requests(self) -> List[RobotRequest]:
        map_info = self.map_provider.get_grid_info()
        if map_info is None:
            return []

        requests: List[RobotRequest] = []

        # Берём все pending задачи.
        # Для первого рабочего варианта replanning идёт по новым задачам.
        for robot_name, goal_name in self.pending_tasks.items():
            robot = self.robots[robot_name]
            pose = robot.get_pose()
            if pose is None:
                self.get_logger().warn(f"[{robot_name}] no pose yet, skip request build")
                continue

            lm = self.landmarks[goal_name]

            start_cell = world_to_grid(pose.x, pose.y, map_info)
            goal_cell = world_to_grid(lm["x"], lm["y"], map_info)

            if start_cell is None:
                self.get_logger().error(f"[{robot_name}] start is outside map")
                continue

            if goal_cell is None:
                self.get_logger().error(f"[{robot_name}] goal {goal_name} is outside map")
                continue

            if not self.map_provider.is_free(goal_cell[0], goal_cell[1]):
                self.get_logger().warn(
                    f"[{robot_name}] goal cell {goal_cell} not free, searching nearest free cell"
                )
                adjusted_goal = find_nearest_free_cell(
                    goal_cell,
                    self.map_provider.is_free,
                    max_radius=10,
                )
                if adjusted_goal is None:
                    self.get_logger().error(
                        f"[{robot_name}] no free cell found near goal {goal_cell}"
                    )
                    continue
                self.get_logger().warn(
                    f"[{robot_name}] goal adjusted: {goal_cell} -> {adjusted_goal}"
                )
                goal_cell = adjusted_goal

            self.get_logger().info(
                f"[{robot_name}] CBS request: start_world=({pose.x:.2f}, {pose.y:.2f}) "
                f"start_grid={start_cell}, goal_world=({lm['x']:.2f}, {lm['y']:.2f}) "
                f"goal_grid={goal_cell}, goal_name={goal_name}"
            )

            requests.append(
                RobotRequest(
                    robot_name=robot_name,
                    start=start_cell,
                    goal=goal_cell,
                )
            )

        return requests

    def _apply_cbs_result(self, robot_requests: List[RobotRequest]):
        planner_result = self.cbs_planner.plan_for_robots(robot_requests)

        if not planner_result.plans:
            self.get_logger().error(
                f"CBS failed: reason={planner_result.debug.reason}, "
                f"conflicts_resolved={planner_result.debug.conflicts_resolved}, "
                f"high_level_nodes={planner_result.debug.high_level_nodes}"
            )
            return

        self.get_logger().info(
            f"CBS success: reason={planner_result.debug.reason}, "
            f"conflicts_resolved={planner_result.debug.conflicts_resolved}, "
            f"high_level_nodes={planner_result.debug.high_level_nodes}"
        )

        map_info = self.map_provider.get_grid_info()
        assert map_info is not None

        # Сначала подготавливаем очереди waypoint-ов для всех роботов
        for req in robot_requests:
            robot_name = req.robot_name
            plan = planner_result.plans[robot_name]

            compressed_path = compress_grid_path(plan.path)

            robot_pose = self.robots[robot_name].get_pose()
            if robot_pose is None:
                self.get_logger().warn(f"[{robot_name}] pose disappeared before applying plan")
                continue

            world_waypoints: List[Tuple[float, float]] = []
            for i, j in compressed_path:
                wx, wy = grid_to_world(i, j, map_info)
                world_waypoints.append((wx, wy))

            if not world_waypoints:
                self.get_logger().error(f"[{robot_name}] empty CBS waypoint queue")
                continue

            filtered_waypoints: List[Tuple[float, float]] = []
            for wx, wy in world_waypoints:
                dist = ((wx - robot_pose.x) ** 2 + (wy - robot_pose.y) ** 2) ** 0.5
                if dist > 0.10:
                    filtered_waypoints.append((wx, wy))

            if not filtered_waypoints:
                filtered_waypoints = [world_waypoints[-1]]

            goal_name = self.pending_tasks[robot_name]
            self.active_goal_for_robot[robot_name] = goal_name
            self.robot_waypoint_queue[robot_name] = filtered_waypoints

            self.get_logger().info(
                f"[{robot_name}] CBS path cells={len(plan.path)}, "
                f"compressed={len(compressed_path)}, "
                f"waypoints={self.robot_waypoint_queue[robot_name]}"
            )

        # Потом одновременно запускаем первый waypoint всем участникам плана
        planned_robot_names = [req.robot_name for req in robot_requests]
        for robot_name in planned_robot_names:
            if self.robot_waypoint_queue.get(robot_name):
                self._send_next_waypoint(robot_name)

        # После успешного применения убираем задачи из pending
        for robot_name in planned_robot_names:
            if robot_name in self.pending_tasks:
                del self.pending_tasks[robot_name]

    def _try_plan_pending_tasks_with_cbs(self):
        if not self.pending_tasks:
            return

        robot_requests = self._build_robot_requests()
        if not robot_requests:
            return

        # Для первого рабочего режима:
        # планируем только тех роботов, которые сейчас не исполняют путь.
        ready_requests: List[RobotRequest] = []
        for req in robot_requests:
            status = self.robots[req.robot_name].get_status()
            if status in ["idle", "succeeded", "canceled", "rejected", "aborted"]:
                ready_requests.append(req)
            else:
                self.get_logger().info(
                    f"[{req.robot_name}] skip CBS now, current status={status}"
                )

        if not ready_requests:
            return

        self._apply_cbs_result(ready_requests)

    def _tick(self):
        if not self.map_provider.has_map():
            self.get_logger().info("Waiting for /robot1/map ...")
            return

        map_info = self.map_provider.get_grid_info()
        assert map_info is not None

        for name, robot in self.robots.items():
            pose = robot.get_pose()
            status = robot.get_status()
            prev_status = self.robot_last_status.get(name, "unknown")

            if pose is None:
                self.get_logger().info(f"[{name}] waiting for amcl_pose...")
                self.robot_last_status[name] = status
                continue

            robot_cell = world_to_grid(pose.x, pose.y, map_info)

            log_msg = f"[{name}] pose: x={pose.x:.2f}, y={pose.y:.2f}, status={status}"
            if robot_cell is not None:
                log_msg += f", grid={robot_cell}"
            else:
                log_msg += ", grid=OUT_OF_MAP"

            if name in self.active_goal_for_robot:
                goal_name = self.active_goal_for_robot[name]
                lm = self.landmarks[goal_name]
                goal_cell = world_to_grid(lm["x"], lm["y"], map_info)
                log_msg += f", active_goal={goal_name}, goal_grid={goal_cell}"

            self.get_logger().info(log_msg)

            if prev_status != status:
                self.get_logger().info(f"[{name}] status changed: {prev_status} -> {status}")

                if status == "succeeded":
                    self._on_robot_succeeded(name)
                elif status in ["canceled", "rejected", "aborted"]:
                    self.get_logger().warn(f"[{name}] goal ended with status={status}")
                    self._clear_robot_task_state(name)

            self.robot_last_status[name] = status

        # После обработки статусов пробуем запланировать новые задачи совместно
        self._try_plan_pending_tasks_with_cbs()


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()