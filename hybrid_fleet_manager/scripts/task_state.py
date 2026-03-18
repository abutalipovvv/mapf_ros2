from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

TaskRef = Tuple[str, Optional[str]]


class TaskStateStore:
    def __init__(self, robot_names: List[str]):
        self.pending_tasks: Dict[str, List[TaskRef]] = {
            name: [] for name in robot_names
        }
        self.active_goal_for_robot: Dict[str, str] = {}
        self.active_task_id_for_robot: Dict[str, Optional[str]] = {}
        self.task_requeue_attempts: Dict[str, int] = {}
        self.cancel_reason_for_robot: Dict[str, str] = {}
        self.pending_blocked_since_sec: Dict[str, float] = {}
        self.pending_blocked_reason: Dict[str, str] = {}

    @staticmethod
    def pending_task_key(robot_name: str, goal_name: str, task_id: Optional[str]) -> str:
        return task_id or f"{robot_name}:{goal_name}"

    def enqueue_pending_task(
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

    def peek_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        queue = self.pending_tasks.get(robot_name, [])
        return queue[0] if queue else None

    def pop_pending_task(self, robot_name: str) -> Optional[TaskRef]:
        queue = self.pending_tasks.get(robot_name, [])
        if not queue:
            return None
        item = queue.pop(0)
        key = self.pending_task_key(robot_name, item[0], item[1])
        self.pending_blocked_since_sec.pop(key, None)
        self.pending_blocked_reason.pop(key, None)
        return item

    def mark_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
        reason: str,
        now_sec: float,
        publish_task_status: Callable[..., None],
    ) -> None:
        key = self.pending_task_key(robot_name, goal_name, task_id)
        if key not in self.pending_blocked_since_sec:
            self.pending_blocked_since_sec[key] = now_sec
        last_reason = self.pending_blocked_reason.get(key)
        if last_reason != reason:
            self.pending_blocked_reason[key] = reason
            publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="blocked",
                reason=reason,
                task_id=task_id,
            )

    def clear_pending_blocked(
        self,
        robot_name: str,
        goal_name: str,
        task_id: Optional[str],
    ) -> None:
        key = self.pending_task_key(robot_name, goal_name, task_id)
        self.pending_blocked_since_sec.pop(key, None)
        self.pending_blocked_reason.pop(key, None)

    def expire_blocked_pending_tasks(
        self,
        robot_names: List[str],
        now_sec: float,
        pending_blocked_timeout_sec: float,
        publish_task_status: Callable[..., None],
    ) -> None:
        if pending_blocked_timeout_sec <= 0.0:
            return

        for robot_name in robot_names:
            pending_task = self.peek_pending_task(robot_name)
            if pending_task is None:
                continue
            goal_name, task_id = pending_task
            key = self.pending_task_key(robot_name, goal_name, task_id)
            blocked_since = self.pending_blocked_since_sec.get(key)
            if blocked_since is None:
                continue
            elapsed = now_sec - blocked_since
            if elapsed < pending_blocked_timeout_sec:
                continue

            popped = self.pop_pending_task(robot_name)
            if popped is None:
                continue

            publish_task_status(
                robot_name=robot_name,
                goal_name=goal_name,
                state="failed",
                reason=f"blocked_timeout_{elapsed:.1f}s",
                task_id=task_id,
            )

    def has_pending_tasks(self, robot_names: List[str]) -> bool:
        return any(self.pending_tasks.get(name, []) for name in robot_names)

    def active_goal_owner(self, goal_name: str, except_robot: str) -> Optional[str]:
        for robot_name, active_goal in self.active_goal_for_robot.items():
            if robot_name == except_robot:
                continue
            if active_goal == goal_name:
                return robot_name
        return None
