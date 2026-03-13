from __future__ import annotations

import math
from typing import Tuple


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw angle (rad) into quaternion (x, y, z, w)
    for planar rotation around Z axis.
    """
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw