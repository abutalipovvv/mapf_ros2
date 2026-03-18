from __future__ import annotations

import heapq
import math
from typing import Dict, List, Optional, Set, Tuple

from scripts.utils.grid_utils import grid_to_world, world_to_grid

GridCell = Tuple[int, int]
WorldPoint = Tuple[float, float]
LandmarkEdge = Tuple[str, str]


class LandmarkRouter:
    def __init__(self, landmarks: Dict[str, Dict[str, float]], default_capture_radius_m: float = 0.35):
        self.landmarks = landmarks
        self.default_capture_radius_m = max(0.05, float(default_capture_radius_m))
        self.graph = self._build_graph()
        self.graph_edges = self._build_graph_edges()

    def _build_graph(self) -> Dict[str, List[str]]:
        grouped_by_y: Dict[float, List[Tuple[float, str]]] = {}
        grouped_by_x: Dict[float, List[Tuple[float, str]]] = {}

        for name, lm in self.landmarks.items():
            x = round(float(lm["x"]), 4)
            y = round(float(lm["y"]), 4)
            grouped_by_y.setdefault(y, []).append((x, name))
            grouped_by_x.setdefault(x, []).append((y, name))

        graph: Dict[str, Set[str]] = {name: set() for name in self.landmarks}

        for items in grouped_by_y.values():
            items.sort()
            for idx in range(len(items) - 1):
                a = items[idx][1]
                b = items[idx + 1][1]
                graph[a].add(b)
                graph[b].add(a)

        for items in grouped_by_x.values():
            items.sort()
            for idx in range(len(items) - 1):
                a = items[idx][1]
                b = items[idx + 1][1]
                graph[a].add(b)
                graph[b].add(a)

        return {
            name: sorted(neighbors)
            for name, neighbors in graph.items()
        }

    def _build_graph_edges(self) -> List[LandmarkEdge]:
        edge_set: Set[LandmarkEdge] = set()
        for name, neighbors in self.graph.items():
            for neighbor in neighbors:
                edge_set.add(tuple(sorted((name, neighbor))))
        return sorted(edge_set)

    def nearest_landmark_name(
        self,
        x: float,
        y: float,
        max_distance: float,
    ) -> Optional[str]:
        radius = max(0.0, float(max_distance))
        best_name: Optional[str] = None
        best_distance_sq = radius * radius

        for name, lm in self.landmarks.items():
            dx = float(lm["x"]) - x
            dy = float(lm["y"]) - y
            dist_sq = dx * dx + dy * dy
            if dist_sq <= best_distance_sq:
                best_distance_sq = dist_sq
                best_name = name

        return best_name

    def landmark_distance(self, a_name: str, b_name: str) -> float:
        a = self.landmarks[a_name]
        b = self.landmarks[b_name]
        return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"]))

    def plan_route(
        self,
        start_name: str,
        goal_name: str,
        blocked_landmarks: Set[str],
    ) -> Optional[List[str]]:
        if start_name not in self.graph or goal_name not in self.graph:
            return None
        if start_name == goal_name:
            return [start_name]
        if goal_name in blocked_landmarks:
            return None

        frontier: List[Tuple[float, str]] = [(0.0, start_name)]
        came_from: Dict[str, Optional[str]] = {start_name: None}
        cost_so_far: Dict[str, float] = {start_name: 0.0}

        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal_name:
                break

            for neighbor in self.graph.get(current, []):
                if neighbor != goal_name and neighbor in blocked_landmarks:
                    continue
                new_cost = cost_so_far[current] + self.landmark_distance(current, neighbor)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self.landmark_distance(neighbor, goal_name)
                    heapq.heappush(frontier, (priority, neighbor))
                    came_from[neighbor] = current

        if goal_name not in came_from:
            return None

        route: List[str] = []
        current: Optional[str] = goal_name
        while current is not None:
            route.append(current)
            current = came_from[current]
        route.reverse()
        return route

    def landmark_name_for_cell(self, cell: GridCell, map_info) -> Optional[str]:
        wx, wy = grid_to_world(cell[0], cell[1], map_info)
        return self.nearest_landmark_name(wx, wy, max_distance=max(map_info.resolution * 1.5, 0.2))

    def format_blocked_cells_for_log(self, blocked_cells: List[GridCell], map_info) -> str:
        by_landmark: Dict[str, int] = {}
        unmapped = 0

        for cell in blocked_cells:
            lm_name = self.landmark_name_for_cell(cell, map_info)
            if lm_name is None:
                unmapped += 1
                continue
            by_landmark[lm_name] = by_landmark.get(lm_name, 0) + 1

        landmark_summary = ", ".join(
            f"{name}:{count}"
            for name, count in sorted(by_landmark.items())
        ) or "none"

        return (
            f"count={len(blocked_cells)}, "
            f"near_landmarks={{ {landmark_summary} }}, "
            f"unmapped={unmapped}"
        )

    def landmark_sequence_from_grid_path(
        self,
        path: List[GridCell],
        map_info,
        capture_radius_m: Optional[float] = None,
    ) -> List[str]:
        if not path:
            return []

        landmark_cells: List[Tuple[str, GridCell]] = []
        for lm_name, lm in self.landmarks.items():
            lm_cell = world_to_grid(float(lm["x"]), float(lm["y"]), map_info)
            if lm_cell is not None:
                landmark_cells.append((lm_name, lm_cell))

        if not landmark_cells:
            return []

        radius_m = self.default_capture_radius_m if capture_radius_m is None else max(0.05, float(capture_radius_m))
        capture_radius_cells = max(
            1,
            int(math.ceil(radius_m / max(map_info.resolution, 1e-6))),
        )

        sequence: List[str] = []
        for cell in path:
            best_name: Optional[str] = None
            best_dist_sq: Optional[int] = None
            for lm_name, lm_cell in landmark_cells:
                dx = cell[0] - lm_cell[0]
                dy = cell[1] - lm_cell[1]
                if max(abs(dx), abs(dy)) > capture_radius_cells:
                    continue
                dist_sq = dx * dx + dy * dy
                if best_dist_sq is None or dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
                    best_name = lm_name
            if best_name is None:
                continue
            if not sequence or sequence[-1] != best_name:
                sequence.append(best_name)
        return sequence

    def landmark_sequence_from_points(
        self,
        points: List[WorldPoint],
        max_distance: float = 0.9,
    ) -> List[str]:
        sequence: List[str] = []
        for x, y in points:
            name = self.nearest_landmark_name(x, y, max_distance=max_distance)
            if name is None:
                continue
            if not sequence or sequence[-1] != name:
                sequence.append(name)
        return sequence

    def compute_view_bounds(self, margin: float = 1.0) -> Tuple[float, float, float, float]:
        if not self.landmarks:
            return (-5.0, 5.0, -5.0, 5.0)

        xs = [float(lm["x"]) for lm in self.landmarks.values()]
        ys = [float(lm["y"]) for lm in self.landmarks.values()]
        return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)
