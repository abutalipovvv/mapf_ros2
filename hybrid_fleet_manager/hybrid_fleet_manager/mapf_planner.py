from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from hybrid_fleet_manager.a_star_grid import a_star_grid, AStarDebugInfo

GridCell = Tuple[int, int]


@dataclass
class RobotPlan:
    robot_name: str
    start: GridCell
    goal: GridCell
    path: List[GridCell]


@dataclass
class PlannerResult:
    plan: Optional[RobotPlan]
    debug: AStarDebugInfo


class MapfPlanner:
    def __init__(self, is_free_fn):
        self.is_free_fn = is_free_fn

    def plan_for_robot(
        self,
        robot_name: str,
        start: GridCell,
        goal: GridCell,
    ) -> PlannerResult:
        path, debug = a_star_grid(
            start=start,
            goal=goal,
            is_free_fn=self.is_free_fn,
        )

        if path is None:
            return PlannerResult(plan=None, debug=debug)

        return PlannerResult(
            plan=RobotPlan(
                robot_name=robot_name,
                start=start,
                goal=goal,
                path=path,
            ),
            debug=debug,
        )