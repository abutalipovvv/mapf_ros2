from __future__ import annotations

from typing import Dict, List, Tuple

try:
    from scripts.planning.cbs_planner import CBSPlanner, RobotRequest
except ModuleNotFoundError:
    from scripts.planning.cbs_planner import CBSPlanner, RobotRequest


GridCell = Tuple[int, int]


def bounded_free(width: int, height: int, blocked: List[GridCell] | None = None):
    blocked_set = set(blocked or [])

    def is_free(i: int, j: int) -> bool:
        return 0 <= i < width and 0 <= j < height and (i, j) not in blocked_set

    return is_free


def state_at(path: List[GridCell], t: int) -> GridCell:
    if t < len(path):
        return path[t]
    return path[-1]


def print_solution(title: str, plans: Dict[str, List[GridCell]]) -> None:
    print(f"\n=== {title} ===")
    for name, path in plans.items():
        print(f"{name}: {path}")

    max_t = max(len(path) for path in plans.values())
    print("timeline:")
    for t in range(max_t):
        snapshot = ", ".join(
            f"{name}={state_at(path, t)}"
            for name, path in sorted(plans.items())
        )
        print(f"  t={t}: {snapshot}")


def run_case(title: str, planner: CBSPlanner, requests: List[RobotRequest]) -> None:
    result = planner.plan_for_robots(requests)
    print(f"\n[{title}] reason={result.debug.reason}")
    print(
        "debug: "
        f"conflicts_resolved={result.debug.conflicts_resolved}, "
        f"high_level_nodes={result.debug.high_level_nodes}"
    )
    if not result.plans:
        return
    print_solution(title, {name: plan.path for name, plan in result.plans.items()})


def main() -> None:
    run_case(
        "intersection_crossing",
        CBSPlanner(bounded_free(5, 5)),
        [
            RobotRequest("robot1", (0, 2), (4, 2)),
            RobotRequest("robot2", (2, 0), (2, 4)),
        ],
    )

    run_case(
        "side_pocket_swap",
        CBSPlanner(bounded_free(3, 3)),
        [
            RobotRequest("robot1", (0, 1), (2, 1)),
            RobotRequest("robot2", (2, 1), (0, 1)),
        ],
    )

    run_case(
        "three_robot_bottleneck",
        CBSPlanner(
            bounded_free(
                5,
                5,
                blocked=[(2, 0), (2, 1), (2, 3), (2, 4)],
            )
        ),
        [
            RobotRequest("robot1", (0, 2), (4, 2)),
            RobotRequest("robot2", (4, 1), (0, 1)),
            RobotRequest("robot3", (4, 3), (0, 3)),
        ],
    )


if __name__ == "__main__":
    main()
