from __future__ import annotations

import math
from typing import Dict, List

from scripts.utils.landmark_loader import load_landmarks

from ..models import ExperimentResult, Scenario


def _assignment_commands(
    mode: str,
    scenario: Scenario,
    landmarks: Dict[str, Dict[str, float]],
) -> List[str]:
    commands: List[str] = []
    if mode == "hybrid":
        for assignment in scenario.assignments:
            commands.append(
                f"ros2 run task_manager task_manager --robot {assignment.robot} --goal {assignment.goal}"
            )
        return commands

    for assignment in scenario.assignments:
        goal = landmarks[assignment.goal]
        yaw = float(goal.get("yaw", 0.0))
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        commands.append(
            "ros2 action send_goal "
            f"/{assignment.robot}/navigate_to_pose "
            "nav2_msgs/action/NavigateToPose "
            f"\"{{pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {goal['x']}, y: {goal['y']}, z: 0.0}}, "
            f"orientation: {{x: 0.0, y: 0.0, z: {qz:.6f}, w: {qw:.6f}}}}}}}}}\""
        )
    return commands


def run(
    mode: str,
    scenario: Scenario,
    landmarks_file: str,
    run_index: int,
    result_status: str,
    execution_time_sec: float,
    success_rate: float,
    blocked_tasks: int,
    requeue_count: int,
    notes: str,
) -> ExperimentResult:
    landmarks = load_landmarks(landmarks_file)

    return ExperimentResult(
        mode=mode,
        scenario=scenario.name,
        run_index=run_index,
        status=result_status,
        metrics={
            "robot_count": len(scenario.assignments),
            "planning_time_sec": 0.0,
            "execution_time_sec": execution_time_sec,
            "success_rate": success_rate,
            "completed_tasks": int(round(success_rate * len(scenario.assignments))),
            "failed_tasks": max(0, len(scenario.assignments) - int(round(success_rate * len(scenario.assignments)))),
            "blocked_tasks": blocked_tasks,
            "requeue_count": requeue_count,
            "notes": notes or "manual_run",
        },
        artifacts={
            "description": scenario.description,
            "commands": _assignment_commands(mode, scenario, landmarks),
            "assignments": [
                {
                    "robot": assignment.robot,
                    "start": assignment.start,
                    "goal": assignment.goal,
                }
                for assignment in scenario.assignments
            ],
        },
    )
