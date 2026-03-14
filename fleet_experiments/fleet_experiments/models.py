from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass(frozen=True)
class ScenarioAssignment:
    robot: str
    start: str
    goal: str


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    assignments: List[ScenarioAssignment]
    obstacles: List[str] = field(default_factory=list)


@dataclass
class ExperimentResult:
    mode: str
    scenario: str
    run_index: int
    status: str
    metrics: Dict[str, Any] = field(default_factory=dict)
    artifacts: Dict[str, Any] = field(default_factory=dict)
