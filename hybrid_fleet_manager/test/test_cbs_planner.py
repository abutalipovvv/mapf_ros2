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
    max_t = max(len(p) for p in paths.values())
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


def test_cbs_independent_paths_success():
    planner = CBSPlanner(_bounded_free(6, 6))
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (0, 0), (3, 0)),
            RobotRequest("robot2", (0, 2), (3, 2)),
        ]
    )

    assert result.debug.reason == "success"
    assert set(result.plans.keys()) == {"robot1", "robot2"}
    assert result.plans["robot1"].path[0] == (0, 0)
    assert result.plans["robot1"].path[-1] == (3, 0)
    assert result.plans["robot2"].path[0] == (0, 2)
    assert result.plans["robot2"].path[-1] == (3, 2)
    _assert_conflict_free({k: v.path for k, v in result.plans.items()})


def test_cbs_swap_positions_success_conflict_free():
    planner = CBSPlanner(_bounded_free(5, 5))
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (1, 1), (2, 1)),
            RobotRequest("robot2", (2, 1), (1, 1)),
        ]
    )

    assert result.debug.reason == "success"
    assert result.debug.conflicts_resolved > 0
    _assert_conflict_free({k: v.path for k, v in result.plans.items()})


def test_cbs_shared_goal_is_rejected():
    planner = CBSPlanner(_bounded_free(5, 5))
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (0, 0), (2, 2)),
            RobotRequest("robot2", (4, 4), (2, 2)),
        ]
    )

    assert not result.plans
    assert result.debug.reason.startswith("shared_goal_not_supported:")


def test_cbs_respects_reserved_vertex_constraints():
    planner = CBSPlanner(_bounded_free(5, 3))
    result = planner.plan_for_robots(
        [RobotRequest("robot1", (0, 0), (3, 0))],
        reserved_vertex_constraints=[(1, (1, 0)), (2, (1, 0))],
    )

    assert result.debug.reason == "success"
    path = result.plans["robot1"].path
    assert _state_at(path, 1) != (1, 0)
    assert _state_at(path, 2) != (1, 0)
    assert path[-1] == (3, 0)


def test_cbs_respects_reserved_edge_constraints():
    planner = CBSPlanner(_bounded_free(3, 1))
    result = planner.plan_for_robots(
        [RobotRequest("robot1", (1, 0), (0, 0))],
        reserved_edge_constraints=[(0, (1, 0), (0, 0))],
    )

    assert result.debug.reason == "success"
    path = result.plans["robot1"].path
    assert _state_at(path, 1) == (1, 0)
    assert path[-1] == (0, 0)


def test_cbs_start_reserved_is_rejected():
    planner = CBSPlanner(_bounded_free(3, 1))
    result = planner.plan_for_robots(
        [RobotRequest("robot1", (1, 0), (2, 0))],
        reserved_vertex_constraints=[(0, (1, 0))],
    )

    assert not result.plans
    assert result.debug.reason == "start_reserved:robot1"


def test_cbs_high_level_node_limit_stops_search():
    planner = CBSPlanner(
        _bounded_free(5, 5),
        max_high_level_nodes=1,
        max_planning_time_sec=5.0,
    )
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (1, 1), (2, 1)),
            RobotRequest("robot2", (2, 1), (1, 1)),
        ]
    )

    assert not result.plans
    assert result.debug.reason == "high_level_node_limit:1"


def test_cbs_planning_timeout_stops_search():
    planner = CBSPlanner(
        _bounded_free(5, 5),
        max_high_level_nodes=2000,
        max_planning_time_sec=0.0,
    )
    result = planner.plan_for_robots(
        [
            RobotRequest("robot1", (1, 1), (2, 1)),
            RobotRequest("robot2", (2, 1), (1, 1)),
        ]
    )

    assert not result.plans
    assert result.debug.reason.startswith("planning_timeout:")
