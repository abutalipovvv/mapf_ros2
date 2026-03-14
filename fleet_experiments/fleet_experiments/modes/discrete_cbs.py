from __future__ import annotations

from dataclasses import asdict
from pathlib import Path
import subprocess
import sys
import time
from typing import Dict, Tuple

import yaml

from scripts.utils.landmark_loader import load_landmarks

from ..models import ExperimentResult, Scenario


GridCell = Tuple[int, int]


def _build_landmark_grid(
    landmarks: Dict[str, Dict[str, float]],
) -> tuple[Dict[str, GridCell], Dict[GridCell, str], tuple[int, int]]:
    xs = sorted({float(item["x"]) for item in landmarks.values()})
    ys = sorted({float(item["y"]) for item in landmarks.values()})

    x_to_ix = {x: ix for ix, x in enumerate(xs)}
    y_to_iy = {y: iy for iy, y in enumerate(ys)}

    lm_to_cell: Dict[str, GridCell] = {}
    cell_to_lm: Dict[GridCell, str] = {}
    for name, item in landmarks.items():
        cell = (x_to_ix[float(item["x"])], y_to_iy[float(item["y"])])
        lm_to_cell[name] = cell
        cell_to_lm[cell] = name

    return lm_to_cell, cell_to_lm, (len(xs), len(ys))


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _planner_paths() -> tuple[Path, Path]:
    repo_root = _repo_root()
    cbs_script = repo_root / "multi_agent_path_planning" / "centralized" / "cbs" / "cbs.py"
    visualize_script = repo_root / "multi_agent_path_planning" / "centralized" / "visualize.py"
    return cbs_script, visualize_script


def run(
    mode: str,
    scenario: Scenario,
    landmarks_file: str,
    run_index: int,
    results_root: str,
    visualize: bool = False,
) -> ExperimentResult:
    landmarks = load_landmarks(landmarks_file)
    lm_to_cell, cell_to_lm, dimensions = _build_landmark_grid(landmarks)

    input_payload = {
        "agents": [
            {
                "name": assignment.robot,
                "start": list(lm_to_cell[assignment.start]),
                "goal": list(lm_to_cell[assignment.goal]),
            }
            for assignment in scenario.assignments
        ],
        "map": {
            "dimensions": list(dimensions),
            "obstacles": [
                list(lm_to_cell[name])
                for name in scenario.obstacles
                if name in lm_to_cell
            ],
        },
    }

    raw_dir = Path(results_root) / "raw" / "discrete_cbs"
    raw_dir.mkdir(parents=True, exist_ok=True)
    input_path = raw_dir / f"{scenario.name}_run{run_index:02d}_input.yaml"
    output_path = raw_dir / f"{scenario.name}_run{run_index:02d}_output.yaml"

    with input_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(input_payload, f, sort_keys=False)

    cbs_script, visualize_script = _planner_paths()
    planner_cwd = cbs_script.parent

    planning_start = time.monotonic()
    completed = subprocess.run(
        [sys.executable, str(cbs_script), str(input_path), str(output_path)],
        cwd=str(planner_cwd),
        capture_output=True,
        text=True,
        check=False,
    )
    planning_time_sec = time.monotonic() - planning_start

    if completed.returncode != 0 or not output_path.exists():
        return ExperimentResult(
            mode=mode,
            scenario=scenario.name,
            run_index=run_index,
            status="failed",
            metrics={
                "robot_count": len(scenario.assignments),
                "planning_time_sec": round(planning_time_sec, 6),
                "success_rate": 0.0,
                "completed_tasks": 0,
                "failed_tasks": len(scenario.assignments),
                "notes": "external_cbs_failed",
            },
            artifacts={
                "input_yaml": str(input_path),
                "output_yaml": str(output_path),
                "planner_stdout": completed.stdout,
                "planner_stderr": completed.stderr,
                "visualize_command": (
                    f"{sys.executable} {visualize_script} {input_path} {output_path}"
                ),
            },
        )

    with output_path.open("r", encoding="utf-8") as f:
        output_payload = yaml.safe_load(f)

    schedule = output_payload.get("schedule", {})
    cost = int(output_payload.get("cost", 0))
    makespan = 0
    discrete_paths: Dict[str, list[str]] = {}
    for robot_name, states in schedule.items():
        makespan = max(makespan, max((int(state["t"]) for state in states), default=0))
        discrete_paths[robot_name] = [
            cell_to_lm[(int(state["x"]), int(state["y"]))] for state in states
        ]

    visualize_command = f"{sys.executable} {visualize_script} {input_path} {output_path}"
    visualize_returncode = None
    if visualize:
        visualize_proc = subprocess.run(
            [sys.executable, str(visualize_script), str(input_path), str(output_path)],
            cwd=str(visualize_script.parent),
            capture_output=False,
            text=True,
            check=False,
        )
        visualize_returncode = visualize_proc.returncode

    return ExperimentResult(
        mode=mode,
        scenario=scenario.name,
        run_index=run_index,
        status="completed",
        metrics={
            "robot_count": len(scenario.assignments),
            "makespan": makespan,
            "sum_of_costs": cost,
            "planning_time_sec": round(planning_time_sec, 6),
            "execution_time_sec": 0.0,
            "success_rate": 1.0,
            "completed_tasks": len(scenario.assignments),
            "failed_tasks": 0,
            "blocked_tasks": 0,
            "requeue_count": 0,
            "notes": "external_cbs_reference",
        },
        artifacts={
            "input_yaml": str(input_path),
            "output_yaml": str(output_path),
            "planner_stdout": completed.stdout,
            "planner_stderr": completed.stderr,
            "discrete_paths": discrete_paths,
            "schedule": schedule,
            "visualize_command": visualize_command,
            "visualize_returncode": visualize_returncode,
            "scenario_description": scenario.description,
            "input_payload": input_payload,
            "output_payload": output_payload,
        },
    )
