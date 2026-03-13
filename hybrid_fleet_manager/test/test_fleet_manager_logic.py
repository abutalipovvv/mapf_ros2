from __future__ import annotations

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if "nav2_msgs" not in sys.modules:
    nav2_msgs_module = types.ModuleType("nav2_msgs")
    nav2_msgs_action_module = types.ModuleType("nav2_msgs.action")

    class _FakeNavigateToPose:
        class Goal:
            pass

    nav2_msgs_action_module.NavigateToPose = _FakeNavigateToPose
    nav2_msgs_module.action = nav2_msgs_action_module
    sys.modules["nav2_msgs"] = nav2_msgs_module
    sys.modules["nav2_msgs.action"] = nav2_msgs_action_module

from scripts.planning.cbs_planner import CBSPlanner, RobotRequest
from scripts.runtime.robot_client import RobotPose
from scripts.utils.grid_utils import GridMapInfo
from src.fleet_manager_node import FleetManagerNode


class _FakeRobot:
    def __init__(self, x: float, y: float, status: str = "idle"):
        self._pose = RobotPose(x=x, y=y, qx=0.0, qy=0.0, qz=0.0, qw=1.0)
        self._status = status

    def get_pose(self):
        return self._pose

    def get_status(self):
        return self._status


class _FakeMapProvider:
    def __init__(self, map_info: GridMapInfo):
        self._map_info = map_info

    def get_grid_info(self):
        return self._map_info

    def is_free(self, i: int, j: int) -> bool:
        return True


class _FakeLogger:
    def info(self, _msg: str) -> None:
        pass


class _StatusRecorder:
    def __init__(self):
        self.events = []

    def __call__(self, **kwargs):
        self.events.append(kwargs)


class _FakePublisher:
    def publish(self, _msg) -> None:
        pass


def test_build_robot_requests_skips_goal_occupied_by_other_robot():
    node = object.__new__(FleetManagerNode)
    node.robot_names = ["robot1", "robot2"]
    node.pending_tasks = {
        "robot1": [("LM4", "task-1")],
        "robot2": [],
    }
    node.pending_blocked_since_sec = {}
    node.pending_blocked_reason = {}
    node._now_sec = lambda: 0.0
    node.task_requeue_attempts = {}
    node.task_status_pub = _FakePublisher()
    node.goal_occupied_radius_m = 0.35
    node.active_goal_for_robot = {}
    node.landmarks = {
        "LM4": {"x": -1.5, "y": 0.5, "yaw": 0.0},
    }
    node.robots = {
        "robot1": _FakeRobot(0.5, 0.5),
        "robot2": _FakeRobot(-1.5, 0.5),
    }
    node.map_provider = _FakeMapProvider(
        GridMapInfo(width=10, height=10, resolution=1.0, origin_x=-2.0, origin_y=0.0)
    )
    node.get_logger = lambda: _FakeLogger()

    requests, goal_info = FleetManagerNode._build_robot_requests(node)

    assert requests == []
    assert goal_info == {}


def test_build_robot_requests_skips_goal_if_other_robot_is_close_to_landmark():
    node = object.__new__(FleetManagerNode)
    node.robot_names = ["robot1", "robot2"]
    node.pending_tasks = {
        "robot1": [("LM1", "task-1")],
        "robot2": [],
    }
    node.pending_blocked_since_sec = {}
    node.pending_blocked_reason = {}
    node._now_sec = lambda: 0.0
    node.task_requeue_attempts = {}
    node.task_status_pub = _FakePublisher()
    node.goal_occupied_radius_m = 0.35
    node.active_goal_for_robot = {}
    node.landmarks = {
        "LM1": {"x": -2.0, "y": 2.0, "yaw": 0.0},
    }
    node.robots = {
        "robot1": _FakeRobot(0.5, 0.5),
        "robot2": _FakeRobot(-1.82, 1.92),
    }
    node.map_provider = _FakeMapProvider(
        GridMapInfo(width=20, height=20, resolution=0.2, origin_x=-3.0, origin_y=0.0)
    )
    node.get_logger = lambda: _FakeLogger()

    requests, goal_info = FleetManagerNode._build_robot_requests(node)

    assert requests == []
    assert goal_info == {}


def test_collect_static_blocked_cells_includes_idle_robot_position():
    node = object.__new__(FleetManagerNode)
    map_info = GridMapInfo(width=10, height=10, resolution=1.0, origin_x=-2.0, origin_y=0.0)

    node.robots = {
        "robot1": _FakeRobot(0.5, 0.5, status="idle"),
        "robot2": _FakeRobot(-0.5, 0.5, status="idle"),
        "robot3": _FakeRobot(1.5, 0.5, status="executing"),
    }
    node.pending_blocked_since_sec = {}
    node.pending_blocked_reason = {}
    node.map_provider = _FakeMapProvider(map_info)

    blocked_cells = FleetManagerNode._collect_static_blocked_cells(
        node,
        excluded_robot_names={"robot1"},
    )

    assert blocked_cells == [(1, 0)]


def test_collect_static_blocked_cells_includes_succeeded_robot_position():
    node = object.__new__(FleetManagerNode)
    map_info = GridMapInfo(width=10, height=10, resolution=1.0, origin_x=-2.0, origin_y=0.0)

    node.robots = {
        "robot1": _FakeRobot(0.5, 0.5, status="idle"),
        "robot2": _FakeRobot(-0.5, 0.5, status="succeeded"),
        "robot3": _FakeRobot(1.5, 0.5, status="goal_sent"),
    }
    node.pending_blocked_since_sec = {}
    node.pending_blocked_reason = {}
    node.map_provider = _FakeMapProvider(map_info)

    blocked_cells = FleetManagerNode._collect_static_blocked_cells(
        node,
        excluded_robot_names={"robot1"},
    )

    assert blocked_cells == [(1, 0)]


def test_cbs_cannot_plan_through_idle_robot_blocked_cell():
    planner = CBSPlanner(lambda i, j: 0 <= i < 5 and 0 <= j < 1)
    result = planner.plan_for_robots(
        [RobotRequest("robot1", (0, 0), (4, 0))],
        blocked_cells=[(2, 0)],
    )

    assert not result.plans
    assert result.debug.reason == "initial_solution_failed"


def test_expire_blocked_pending_task_marks_failed_and_removes_it():
    node = object.__new__(FleetManagerNode)
    node.robot_names = ["robot1"]
    node.pending_tasks = {"robot1": [("LM4", "task-1")]}
    node.pending_blocked_since_sec = {"task-1": 0.0}
    node.pending_blocked_reason = {"task-1": "goal_occupied_by_robot2"}
    node.pending_blocked_timeout_sec = 10.0
    node._now_sec = lambda: 15.0
    recorder = _StatusRecorder()
    node._publish_task_status = recorder

    FleetManagerNode._expire_blocked_pending_tasks(node)

    assert node.pending_tasks["robot1"] == []
    assert recorder.events == [
        {
            "robot_name": "robot1",
            "goal_name": "LM4",
            "state": "failed",
            "reason": "blocked_timeout_15.0s",
            "task_id": "task-1",
        }
    ]


def test_mark_pending_blocked_does_not_reset_wait_timer_when_reason_changes():
    node = object.__new__(FleetManagerNode)
    node.pending_blocked_since_sec = {}
    node.pending_blocked_reason = {}
    node.task_requeue_attempts = {}
    node.task_status_pub = _FakePublisher()
    node.get_logger = lambda: _FakeLogger()

    now_values = iter([5.0, 20.0])
    node._now_sec = lambda: next(now_values)

    FleetManagerNode._mark_pending_blocked(
        node,
        robot_name="robot1",
        goal_name="LM4",
        task_id="task-1",
        reason="goal_active_on_robot2",
    )
    FleetManagerNode._mark_pending_blocked(
        node,
        robot_name="robot1",
        goal_name="LM4",
        task_id="task-1",
        reason="goal_occupied_by_robot2",
    )

    assert node.pending_blocked_since_sec["task-1"] == 5.0
    assert node.pending_blocked_reason["task-1"] == "goal_occupied_by_robot2"
