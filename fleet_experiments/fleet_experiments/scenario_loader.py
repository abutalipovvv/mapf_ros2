from __future__ import annotations

from pathlib import Path
from typing import Dict

import yaml

from .models import Scenario, ScenarioAssignment


def load_scenarios(path: str) -> Dict[str, Scenario]:
    scenario_path = Path(path)
    if not scenario_path.exists():
        raise FileNotFoundError(f"Scenario file not found: {path}")

    with scenario_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict) or not isinstance(data.get("scenarios"), dict):
        raise ValueError("Scenario YAML must contain top-level 'scenarios' mapping")

    scenarios: Dict[str, Scenario] = {}
    for name, raw in data["scenarios"].items():
        if not isinstance(raw, dict):
            raise ValueError(f"Scenario '{name}' must be a mapping")

        description = str(raw.get("description", "")).strip()
        raw_obstacles = raw.get("obstacles", [])
        if raw_obstacles is None:
            raw_obstacles = []
        if not isinstance(raw_obstacles, list):
            raise ValueError(f"Scenario '{name}' obstacles must be a list when provided")
        raw_assignments = raw.get("assignments")
        if not isinstance(raw_assignments, list) or not raw_assignments:
            raise ValueError(f"Scenario '{name}' must define a non-empty assignments list")

        assignments = []
        for item in raw_assignments:
            if not isinstance(item, dict):
                raise ValueError(f"Scenario '{name}' assignment must be a mapping")

            robot = str(item.get("robot", "")).strip()
            start = str(item.get("start", "")).strip()
            goal = str(item.get("goal", "")).strip()
            if not robot or not start or not goal:
                raise ValueError(
                    f"Scenario '{name}' assignments must contain robot, start, goal"
                )
            assignments.append(ScenarioAssignment(robot=robot, start=start, goal=goal))

        scenarios[name] = Scenario(
            name=name,
            description=description,
            assignments=assignments,
            obstacles=[str(item).strip() for item in raw_obstacles if str(item).strip()],
        )

    return scenarios
