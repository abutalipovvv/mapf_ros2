from __future__ import annotations

from typing import Dict, List, Optional, Tuple

GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]


class ExecutionState:
    def __init__(self, robot_names: List[str]):
        self.robot_waypoint_queue: Dict[str, List[WorldPoint]] = {
            name: [] for name in robot_names
        }
        self.robot_waypoint_landmark_queue: Dict[str, List[str]] = {
            name: [] for name in robot_names
        }
        self.robot_planned_landmark_path: Dict[str, List[str]] = {
            name: [] for name in robot_names
        }
        self.robot_current_nav_goal_world: Dict[str, Optional[WorldPoint]] = {
            name: None for name in robot_names
        }
        self.robot_current_nav_goal_sent_time_sec: Dict[str, Optional[float]] = {
            name: None for name in robot_names
        }
        self.robot_active_grid_path: Dict[str, List[GridCell]] = {
            name: [] for name in robot_names
        }
        self.robot_offpath_ticks: Dict[str, int] = {
            name: 0 for name in robot_names
        }

    def clear_robot(self, robot_name: str) -> None:
        self.robot_waypoint_queue[robot_name] = []
        self.robot_waypoint_landmark_queue[robot_name] = []
        self.robot_planned_landmark_path[robot_name] = []
        self.robot_current_nav_goal_world[robot_name] = None
        self.robot_current_nav_goal_sent_time_sec[robot_name] = None
        self.robot_active_grid_path[robot_name] = []
        self.robot_offpath_ticks[robot_name] = 0
