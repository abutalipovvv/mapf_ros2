from __future__ import annotations

from typing import Optional, List

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

from scripts.utils.grid_utils import GridMapInfo

class MapProvider:
    def __init__(
        self,
        node: Node,
        topic: str = "/map",
        occ_threshold: int = 50,
        unknown_is_free: bool = True,
    ):
        self.node = node
        self.topic = topic
        self.occ_threshold = occ_threshold
        self.unknown_is_free = unknown_is_free

        self._map_msg: Optional[OccupancyGrid] = None
        self._grid_info: Optional[GridMapInfo] = None
        self._data: Optional[List[int]] = None

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sub = self.node.create_subscription(
            OccupancyGrid,
            self.topic,
            self._map_callback,
            map_qos,
        )

        self.node.get_logger().info(
            f"MapProvider subscribed to {self.topic} "
            f"(occ_threshold={self.occ_threshold}, unknown_is_free={self.unknown_is_free})"
        )

    def _map_callback(self, msg: OccupancyGrid):
        self._map_msg = msg
        self._grid_info = GridMapInfo(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        self._data = list(msg.data)
        self.node.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height}, "
            f"res={msg.info.resolution}, "
            f"origin=({msg.info.origin.position.x:.3f}, {msg.info.origin.position.y:.3f})"
        )

    def has_map(self) -> bool:
        return self._map_msg is not None and self._grid_info is not None and self._data is not None

    def get_grid_info(self) -> Optional[GridMapInfo]:
        return self._grid_info

    def get_data(self) -> Optional[List[int]]:
        return self._data

    def is_free(self, i: int, j: int, occ_threshold: Optional[int] = None) -> bool:
        if not self.has_map():
            return False

        assert self._grid_info is not None
        if i < 0 or j < 0 or i >= self._grid_info.width or j >= self._grid_info.height:
            return False

        occ = self.get_cell_value(i, j)
        threshold = self.occ_threshold if occ_threshold is None else occ_threshold

        if occ == -1:
            return self.unknown_is_free

        return occ < threshold

    def get_cell_value(self, i: int, j: int) -> int:
        if not self.has_map():
            return 100

        assert self._grid_info is not None
        assert self._data is not None

        if i < 0 or j < 0 or i >= self._grid_info.width or j >= self._grid_info.height:
            return 100

        idx = j * self._grid_info.width + i
        return self._data[idx]


    def count_free_neighbors_4(self, i: int, j: int) -> int:
        candidates = [
            (i + 1, j),
            (i - 1, j),
            (i, j + 1),
            (i, j - 1),
        ]
        cnt = 0
        for ni, nj in candidates:
            if self.is_free(ni, nj):
                cnt += 1
        return cnt


    def debug_window_string(self, center_i: int, center_j: int, radius: int = 5) -> str:
        if not self.has_map():
            return "<no map>"

        assert self._grid_info is not None
        lines = []

        for j in range(center_j - radius, center_j + radius + 1):
            row = []
            for i in range(center_i - radius, center_i + radius + 1):
                if i == center_i and j == center_j:
                    row.append("C")
                    continue

                if i < 0 or j < 0 or i >= self._grid_info.width or j >= self._grid_info.height:
                    row.append(" ")
                    continue

                occ = self.get_cell_value(i, j)
                if occ == -1:
                    row.append("?")
                elif occ >= self.occ_threshold:
                    row.append("#")
                else:
                    row.append(".")
            lines.append("".join(row))

        return "\n".join(lines)
