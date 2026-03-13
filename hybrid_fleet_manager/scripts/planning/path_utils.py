from __future__ import annotations

from typing import List, Tuple

GridCell = Tuple[int, int]


def compress_grid_path(path: List[GridCell]) -> List[GridCell]:
    if not path:
        return []

    if len(path) <= 2:
        return path[:]

    compressed: List[GridCell] = [path[0]]

    prev_dx = path[1][0] - path[0][0]
    prev_dy = path[1][1] - path[0][1]

    for i in range(1, len(path) - 1):
        cur = path[i]
        nxt = path[i + 1]

        dx = nxt[0] - cur[0]
        dy = nxt[1] - cur[1]

        if (dx, dy) != (prev_dx, prev_dy):
            compressed.append(cur)

        prev_dx, prev_dy = dx, dy

    compressed.append(path[-1])
    return compressed