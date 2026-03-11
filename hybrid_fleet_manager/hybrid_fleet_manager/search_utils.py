from __future__ import annotations

from collections import deque
from typing import Callable, Optional, Tuple

GridCell = Tuple[int, int]


def find_nearest_free_cell(
    start: GridCell,
    is_free_fn: Callable[[int, int], bool],
    max_radius: int = 10,
) -> Optional[GridCell]:
    sx, sy = start

    if is_free_fn(sx, sy):
        return start

    visited: set[GridCell] = set()
    q = deque()
    q.append((sx, sy, 0))
    visited.add((sx, sy))

    while q:
        x, y, dist = q.popleft()

        if dist > max_radius:
            continue

        if is_free_fn(x, y):
            return (x, y)

        for nx, ny in (
            (x + 1, y),
            (x - 1, y),
            (x, y + 1),
            (x, y - 1),
        ):
            cell = (nx, ny)
            if cell in visited:
                continue
            visited.add(cell)
            q.append((nx, ny, dist + 1))

    return None