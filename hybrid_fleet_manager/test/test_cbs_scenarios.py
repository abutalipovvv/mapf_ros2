from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from typing import Dict, List, Tuple

try:
    from scripts.planning.cbs_planner import CBSPlanner, RobotRequest
except ModuleNotFoundError:
    from scripts.planning.cbs_planner import CBSPlanner, RobotRequest


GridCell = Tuple[int, int]


def _bounded_free(width: int, height: int, blocked: List[GridCell] | None = None):
    blocked_set = set(blocked or [])

    def _is_free(i: int, j: int) -> bool:
        return 0 <= i < width and 0 <= j < height and (i, j) not in blocked_set

    return _is_free


def _state_at(path: List[GridCell], t: int) -> GridCell:
    if t < len(path):
        return path[t]
    return path[-1]


def _assert_conflict_free(paths: Dict[str, List[GridCell]]) -> None:
    max_t = max(len(path) for path in paths.values())
    names = sorted(paths.keys())

    for t in range(max_t):
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a = names[i]
                b = names[j]
                a_t = _state_at(paths[a], t)
                b_t = _state_at(paths[b], t)
                assert a_t != b_t, f"vertex conflict at t={t}: {a}={a_t}, {b}={b_t}"

                a_t1 = _state_at(paths[a], t + 1)
                b_t1 = _state_at(paths[b], t + 1)
                assert not (a_t == b_t1 and a_t1 == b_t), (
                    f"edge conflict at t={t}: {a} {a_t}->{a_t1}, {b} {b_t}->{b_t1}"
                )


def test_cbs_intersection_crossing_resolves_center_conflict():
    planner = CBSPlanner(_bounded_free(5, 5))
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (0, 2), (4, 2)),
            RobotRequest("robot2", (2, 0), (2, 4)),
        ]
    )

    assert result.debug.reason == "success"
    assert result.debug.conflicts_resolved > 0

    paths = {name: plan.path for name, plan in result.plans.items()}
    _assert_conflict_free(paths)
    assert paths["robot1"][0] == (0, 2)
    assert paths["robot1"][-1] == (4, 2)
    assert paths["robot2"][0] == (2, 0)
    assert paths["robot2"][-1] == (2, 4)


def test_cbs_side_pocket_swap_uses_detour_or_wait():
    free = _bounded_free(3, 3)
    planner = CBSPlanner(free)
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (0, 1), (2, 1)),
            RobotRequest("robot2", (2, 1), (0, 1)),
        ]
    )

    assert result.debug.reason == "success"
    assert result.debug.conflicts_resolved > 0

    paths = {name: plan.path for name, plan in result.plans.items()}
    _assert_conflict_free(paths)

    # In this scenario the optimal conflict resolution usually needs either
    # a wait or a temporary move into the side pocket.
    assert len(paths["robot1"]) > 3 or len(paths["robot2"]) > 3


def test_cbs_three_robot_bottleneck_conflict_free():
    blocked = [
        (2, 0),
        (2, 1),
        (2, 3),
        (2, 4),
    ]
    planner = CBSPlanner(_bounded_free(5, 5, blocked))
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (0, 2), (4, 2)),
            RobotRequest("robot2", (4, 1), (0, 1)),
            RobotRequest("robot3", (4, 3), (0, 3)),
        ]
    )

    assert result.debug.reason == "success"
    assert result.debug.conflicts_resolved > 0

    paths = {name: plan.path for name, plan in result.plans.items()}
    _assert_conflict_free(paths)

    assert paths["robot1"][-1] == (4, 2)
    assert paths["robot2"][-1] == (0, 1)
    assert paths["robot3"][-1] == (0, 3)
