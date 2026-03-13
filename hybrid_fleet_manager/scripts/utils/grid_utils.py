from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class GridMapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


def world_to_grid(x: float, y: float, map_info: GridMapInfo) -> Optional[Tuple[int, int]]:
    """
    Convert world coordinates (map frame) to grid cell indices (i, j).
    i -> column (x direction)
    j -> row    (y direction)
    """
    i = int(math.floor((x - map_info.origin_x) / map_info.resolution))
    j = int(math.floor((y - map_info.origin_y) / map_info.resolution))

    if i < 0 or j < 0 or i >= map_info.width or j >= map_info.height:
        return None

    return i, j


def grid_to_world(i: int, j: int, map_info: GridMapInfo) -> Tuple[float, float]:
    """
    Convert grid cell indices (i, j) to world coordinates (center of cell).
    """
    x = map_info.origin_x + (i + 0.5) * map_info.resolution
    y = map_info.origin_y + (j + 0.5) * map_info.resolution
    return x, y


def flatten_index(i: int, j: int, width: int) -> int:
    return j * width + i