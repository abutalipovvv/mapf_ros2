from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

GridCell = Tuple[int, int]


@dataclass
class AStarDebugInfo:
    success: bool
    reason: str
    expanded_nodes: int
    visited_nodes: int
    start_free: bool
    goal_free: bool


def manhattan(a: GridCell, b: GridCell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(
    came_from: Dict[GridCell, GridCell],
    current: GridCell,
) -> List[GridCell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def get_4_neighbors(cell: GridCell) -> List[GridCell]:
    x, y = cell
    return [
        (x + 1, y),
        (x - 1, y),
        (x, y + 1),
        (x, y - 1),
    ]


def a_star_grid(
    start: GridCell,
    goal: GridCell,
    is_free_fn: Callable[[int, int], bool],
) -> Tuple[Optional[List[GridCell]], AStarDebugInfo]:
    start_free = is_free_fn(start[0], start[1])
    goal_free = is_free_fn(goal[0], goal[1])

    if not start_free:
        return None, AStarDebugInfo(
            success=False,
            reason="start_not_free",
            expanded_nodes=0,
            visited_nodes=0,
            start_free=False,
            goal_free=goal_free,
        )

    if not goal_free:
        return None, AStarDebugInfo(
            success=False,
            reason="goal_not_free",
            expanded_nodes=0,
            visited_nodes=0,
            start_free=True,
            goal_free=False,
        )

    open_heap: List[Tuple[int, int, GridCell]] = []
    heapq.heappush(open_heap, (manhattan(start, goal), 0, start))

    came_from: Dict[GridCell, GridCell] = {}
    g_score: Dict[GridCell, int] = {start: 0}
    closed = set()

    expanded_nodes = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)

        if current in closed:
            continue

        closed.add(current)
        expanded_nodes += 1

        if current == goal:
            return reconstruct_path(came_from, current), AStarDebugInfo(
                success=True,
                reason="ok",
                expanded_nodes=expanded_nodes,
                visited_nodes=len(closed),
                start_free=True,
                goal_free=True,
            )

        current_g = g_score[current]

        for nb in get_4_neighbors(current):
            if nb in closed:
                continue

            if not is_free_fn(nb[0], nb[1]):
                continue

            tentative_g = current_g + 1

            if nb not in g_score or tentative_g < g_score[nb]:
                came_from[nb] = current
                g_score[nb] = tentative_g
                f = tentative_g + manhattan(nb, goal)
                heapq.heappush(open_heap, (f, tentative_g, nb))

    return None, AStarDebugInfo(
        success=False,
        reason="no_path_found",
        expanded_nodes=expanded_nodes,
        visited_nodes=len(closed),
        start_free=True,
        goal_free=True,
    )