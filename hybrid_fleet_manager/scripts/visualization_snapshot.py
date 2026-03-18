from __future__ import annotations

from typing import Dict, List, Optional, Tuple

WorldPoint = Tuple[float, float]


class VisualizationSnapshotBuilder:
    def __init__(self, landmarks: Dict[str, Dict[str, float]]):
        self.landmarks = landmarks

    @staticmethod
    def point_to_payload(point: WorldPoint) -> Dict[str, float]:
        return {"x": float(point[0]), "y": float(point[1])}

    def build(
        self,
        robot_names: List[str],
        robots,
        active_goal_for_robot: Dict[str, str],
        current_nav_goals: Dict[str, Optional[WorldPoint]],
        waypoint_landmark_queues: Dict[str, List[str]],
        planned_landmark_paths: Dict[str, List[str]],
    ) -> Dict[str, Dict[str, object]]:
        robots_payload: Dict[str, Dict[str, object]] = {}

        for robot_name in robot_names:
            pose = robots[robot_name].get_pose()
            current_nav_goal = current_nav_goals.get(robot_name)

            local_path: List[WorldPoint] = []
            if current_nav_goal is not None:
                local_path.append(current_nav_goal)
            for lm_name in waypoint_landmark_queues.get(robot_name, []):
                lm = self.landmarks.get(lm_name)
                if lm is None:
                    continue
                local_path.append((float(lm["x"]), float(lm["y"])))

            global_path: List[WorldPoint] = []
            for lm_name in planned_landmark_paths.get(robot_name, []):
                lm = self.landmarks.get(lm_name)
                if lm is None:
                    continue
                global_path.append((float(lm["x"]), float(lm["y"])))

            robots_payload[robot_name] = {
                "status": robots[robot_name].get_status(),
                "goal": active_goal_for_robot.get(robot_name),
                "pose": (
                    None
                    if pose is None
                    else {"x": float(pose.x), "y": float(pose.y)}
                ),
                "current_nav_goal": (
                    None
                    if current_nav_goal is None
                    else self.point_to_payload(current_nav_goal)
                ),
                "local_path": [self.point_to_payload(point) for point in local_path],
                "global_path": [self.point_to_payload(point) for point in global_path],
            }

        return robots_payload
