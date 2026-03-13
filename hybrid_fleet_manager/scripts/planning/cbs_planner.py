from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from heapq import heappop, heappush
from itertools import combinations
import time as py_time
from typing import Callable, Dict, List, Optional, Tuple

GridCell = Tuple[int, int]


@dataclass(frozen=True)
class Location:
    x: int
    y: int

    def as_tuple(self) -> GridCell:
        return (self.x, self.y)


@dataclass(frozen=True)
class State:
    time: int
    location: Location

    def is_equal_except_time(self, other: "State") -> bool:
        return self.location == other.location


@dataclass
class Conflict:
    VERTEX = 1
    EDGE = 2

    time: int = -1
    type: int = -1
    agent_1: str = ""
    agent_2: str = ""
    location_1: Optional[Location] = None
    location_2: Optional[Location] = None


@dataclass(frozen=True)
class VertexConstraint:
    time: int
    location: Location


@dataclass(frozen=True)
class EdgeConstraint:
    time: int
    location_1: Location
    location_2: Location


@dataclass
class Constraints:
    vertex_constraints: set[VertexConstraint] = field(default_factory=set)
    edge_constraints: set[EdgeConstraint] = field(default_factory=set)

    def add_constraint(self, other: "Constraints") -> None:
        self.vertex_constraints |= other.vertex_constraints
        self.edge_constraints |= other.edge_constraints


@dataclass
class RobotRequest:
    robot_name: str
    start: GridCell
    goal: GridCell


@dataclass
class RobotPlan:
    robot_name: str
    start: GridCell
    goal: GridCell
    path: List[GridCell]


@dataclass
class PlannerDebug:
    reason: str = "unknown"
    expanded_nodes: int = 0
    visited_nodes: int = 0
    start_free: bool = False
    goal_free: bool = False
    conflicts_resolved: int = 0
    high_level_nodes: int = 0


@dataclass
class PlannerResult:
    plans: Dict[str, RobotPlan]
    debug: PlannerDebug


@dataclass
class HighLevelNode:
    solution: Dict[str, List[State]] = field(default_factory=dict)
    constraint_dict: Dict[str, Constraints] = field(default_factory=dict)
    cost: int = 0

    def __hash__(self) -> int:
        return hash(self.cost)

    def __lt__(self, other: "HighLevelNode") -> bool:
        return self.cost < other.cost


class CBSEnvironment:
    def __init__(
        self,
        agent_requests: List[RobotRequest],
        is_free_fn: Callable[[int, int], bool],
        blocked_cells: Optional[set[GridCell]] = None,
        global_vertex_constraints: Optional[set[VertexConstraint]] = None,
        global_edge_constraints: Optional[set[EdgeConstraint]] = None,
        low_level_max_time: int = 256,
    ):
        self.agent_requests = agent_requests
        self.is_free_fn = is_free_fn
        self.blocked_cells = blocked_cells or set()
        self.global_vertex_constraints = global_vertex_constraints or set()
        self.global_edge_constraints = global_edge_constraints or set()
        self.low_level_max_time = low_level_max_time

        self.agent_dict: Dict[str, Dict[str, State]] = {}
        self.constraint_dict: Dict[str, Constraints] = {}
        self.constraints = Constraints()

        self._make_agent_dict()

    def _make_agent_dict(self) -> None:
        for agent in self.agent_requests:
            start_state = State(0, Location(agent.start[0], agent.start[1]))
            goal_state = State(0, Location(agent.goal[0], agent.goal[1]))
            self.agent_dict[agent.robot_name] = {
                "start": start_state,
                "goal": goal_state,
            }

    def admissible_heuristic(self, state: State, agent_name: str) -> int:
        goal = self.agent_dict[agent_name]["goal"]
        return abs(state.location.x - goal.location.x) + abs(state.location.y - goal.location.y)

    def is_at_goal(self, state: State, agent_name: str) -> bool:
        goal_state = self.agent_dict[agent_name]["goal"]
        return state.is_equal_except_time(goal_state)

    def is_goal_state_final(self, state: State, agent_name: str) -> bool:
        """
        Accept goal only if staying at goal will not violate any future constraints.
        """
        if not self.is_at_goal(state, agent_name):
            return False

        constraints = self.constraint_dict.get(agent_name, Constraints())
        goal_location = state.location

        for vc in constraints.vertex_constraints:
            if vc.location == goal_location and vc.time >= state.time:
                return False
        for vc in self.global_vertex_constraints:
            if vc.location == goal_location and vc.time >= state.time:
                return False

        for ec in constraints.edge_constraints:
            if (
                ec.location_1 == goal_location
                and ec.location_2 == goal_location
                and ec.time >= state.time
            ):
                return False
        for ec in self.global_edge_constraints:
            if (
                ec.location_1 == goal_location
                and ec.location_2 == goal_location
                and ec.time >= state.time
            ):
                return False

        return True

    def state_valid(self, state: State) -> bool:
        if not self.is_free_fn(state.location.x, state.location.y):
            return False
        if state.location.as_tuple() in self.blocked_cells:
            return False
        if VertexConstraint(state.time, state.location) in self.constraints.vertex_constraints:
            return False
        if VertexConstraint(state.time, state.location) in self.global_vertex_constraints:
            return False
        return True

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        edge_constraint = EdgeConstraint(state_1.time, state_1.location, state_2.location)
        if edge_constraint in self.constraints.edge_constraints:
            return False
        if edge_constraint in self.global_edge_constraints:
            return False
        return True

    def get_neighbors(self, state: State) -> List[State]:
        neighbors: List[State] = []

        candidates = [
            State(state.time + 1, Location(state.location.x, state.location.y)),      # wait
            State(state.time + 1, Location(state.location.x + 1, state.location.y)),  # right
            State(state.time + 1, Location(state.location.x - 1, state.location.y)),  # left
            State(state.time + 1, Location(state.location.x, state.location.y + 1)),  # up
            State(state.time + 1, Location(state.location.x, state.location.y - 1)),  # down
        ]

        for n in candidates:
            if self.state_valid(n) and self.transition_valid(state, n):
                neighbors.append(n)

        return neighbors

    def get_state(self, agent_name: str, solution: Dict[str, List[State]], t: int) -> State:
        if t < len(solution[agent_name]):
            return solution[agent_name][t]
        return solution[agent_name][-1]

    def get_first_conflict(self, solution: Dict[str, List[State]]) -> Optional[Conflict]:
        max_t = max(len(plan) for plan in solution.values())

        for t in range(max_t):
            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1 = self.get_state(agent_1, solution, t)
                state_2 = self.get_state(agent_2, solution, t)

                if state_1.is_equal_except_time(state_2):
                    return Conflict(
                        time=t,
                        type=Conflict.VERTEX,
                        agent_1=agent_1,
                        agent_2=agent_2,
                        location_1=state_1.location,
                    )

            for agent_1, agent_2 in combinations(solution.keys(), 2):
                state_1a = self.get_state(agent_1, solution, t)
                state_1b = self.get_state(agent_1, solution, t + 1)
                state_2a = self.get_state(agent_2, solution, t)
                state_2b = self.get_state(agent_2, solution, t + 1)

                if (
                    state_1a.is_equal_except_time(state_2b)
                    and state_1b.is_equal_except_time(state_2a)
                ):
                    return Conflict(
                        time=t,
                        type=Conflict.EDGE,
                        agent_1=agent_1,
                        agent_2=agent_2,
                        location_1=state_1a.location,
                        location_2=state_1b.location,
                    )

        return None

    def create_constraints_from_conflict(self, conflict: Conflict) -> Dict[str, Constraints]:
        constraint_dict: Dict[str, Constraints] = {}

        if conflict.type == Conflict.VERTEX:
            assert conflict.location_1 is not None
            v_constraint = VertexConstraint(conflict.time, conflict.location_1)

            constraint = Constraints()
            constraint.vertex_constraints.add(v_constraint)

            constraint_dict[conflict.agent_1] = constraint
            constraint_dict[conflict.agent_2] = constraint

        elif conflict.type == Conflict.EDGE:
            assert conflict.location_1 is not None
            assert conflict.location_2 is not None

            constraint1 = Constraints()
            constraint2 = Constraints()

            e_constraint1 = EdgeConstraint(
                conflict.time,
                conflict.location_1,
                conflict.location_2,
            )
            e_constraint2 = EdgeConstraint(
                conflict.time,
                conflict.location_2,
                conflict.location_1,
            )

            constraint1.edge_constraints.add(e_constraint1)
            constraint2.edge_constraints.add(e_constraint2)

            constraint_dict[conflict.agent_1] = constraint1
            constraint_dict[conflict.agent_2] = constraint2

        return constraint_dict

    def compute_solution(self) -> Optional[Dict[str, List[State]]]:
        solution: Dict[str, List[State]] = {}

        for agent in self.agent_dict.keys():
            self.constraints = self.constraint_dict.setdefault(agent, Constraints())
            local_solution = self.low_level_search(
                agent,
                max_time=self.low_level_max_time,
            )

            if not local_solution:
                return None

            solution[agent] = local_solution

        return solution

    def compute_solution_cost(self, solution: Dict[str, List[State]]) -> int:
        return sum(len(path) for path in solution.values())

    def reconstruct_path(
        self,
        came_from: Dict[State, State],
        current: State,
    ) -> List[State]:
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]

    def low_level_search(self, agent_name: str, max_time: int = 256) -> Optional[List[State]]:
        initial_state = self.agent_dict[agent_name]["start"]
        self.constraints = self.constraint_dict.setdefault(agent_name, Constraints())

        if not self.state_valid(initial_state):
            return None

        open_heap: List[Tuple[int, int, State]] = []
        open_set: set[State] = set()
        closed_set: set[State] = set()

        came_from: Dict[State, State] = {}
        g_score: Dict[State, int] = {initial_state: 0}
        f_score: Dict[State, int] = {
            initial_state: self.admissible_heuristic(initial_state, agent_name)
        }

        counter = 0
        heappush(open_heap, (f_score[initial_state], counter, initial_state))
        open_set.add(initial_state)

        while open_heap:
            _, _, current = heappop(open_heap)
            if current not in open_set:
                continue

            open_set.remove(current)

            if self.is_goal_state_final(current, agent_name):
                return self.reconstruct_path(came_from, current)

            if current.time > max_time:
                continue

            closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + 1

                if tentative_g_score >= g_score.get(neighbor, 10**18):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + self.admissible_heuristic(
                    neighbor, agent_name
                )

                counter += 1
                heappush(open_heap, (f_score[neighbor], counter, neighbor))
                open_set.add(neighbor)

        return None


class CBSPlanner:
    def __init__(
        self,
        is_free_fn: Callable[[int, int], bool],
        low_level_max_time: int = 256,
        max_high_level_nodes: int = 2000,
        max_planning_time_sec: float = 5.0,
    ):
        self.is_free_fn = is_free_fn
        self.low_level_max_time = max(1, int(low_level_max_time))
        self.max_high_level_nodes = max(1, int(max_high_level_nodes))
        self.max_planning_time_sec = max(0.0, float(max_planning_time_sec))

    def plan_for_robots(
        self,
        robot_requests: List[RobotRequest],
        blocked_cells: Optional[List[GridCell]] = None,
        reserved_vertex_constraints: Optional[List[Tuple[int, GridCell]]] = None,
        reserved_edge_constraints: Optional[List[Tuple[int, GridCell, GridCell]]] = None,
        low_level_max_time: Optional[int] = None,
        max_high_level_nodes: Optional[int] = None,
        max_planning_time_sec: Optional[float] = None,
    ) -> PlannerResult:
        debug = PlannerDebug(reason="init")
        blocked_cells = blocked_cells or []
        reserved_vertex_constraints = reserved_vertex_constraints or []
        reserved_edge_constraints = reserved_edge_constraints or []
        ll_max_time = (
            self.low_level_max_time
            if low_level_max_time is None
            else max(1, int(low_level_max_time))
        )
        hl_max_nodes = (
            self.max_high_level_nodes
            if max_high_level_nodes is None
            else max(1, int(max_high_level_nodes))
        )
        planning_time_budget_sec = (
            self.max_planning_time_sec
            if max_planning_time_sec is None
            else max(0.0, float(max_planning_time_sec))
        )
        planning_start_monotonic = py_time.monotonic()

        global_vertex_constraints: set[VertexConstraint] = set()
        for t, cell in reserved_vertex_constraints:
            global_vertex_constraints.add(
                VertexConstraint(
                    time=t,
                    location=Location(cell[0], cell[1]),
                )
            )

        global_edge_constraints: set[EdgeConstraint] = set()
        for t, from_cell, to_cell in reserved_edge_constraints:
            global_edge_constraints.add(
                EdgeConstraint(
                    time=t,
                    location_1=Location(from_cell[0], from_cell[1]),
                    location_2=Location(to_cell[0], to_cell[1]),
                )
            )

        if not robot_requests:
            debug.reason = "empty_requests"
            return PlannerResult(plans={}, debug=debug)

        for req in robot_requests:
            start_free = self.is_free_fn(req.start[0], req.start[1])
            goal_free = self.is_free_fn(req.goal[0], req.goal[1])
            debug.start_free = debug.start_free or start_free
            debug.goal_free = debug.goal_free or goal_free

            if not start_free:
                debug.reason = f"start_not_free:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)

            if not goal_free:
                debug.reason = f"goal_not_free:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)

            if req.start in blocked_cells:
                debug.reason = f"start_blocked:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)

            if req.goal in blocked_cells:
                debug.reason = f"goal_blocked:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)

            if VertexConstraint(0, Location(req.start[0], req.start[1])) in global_vertex_constraints:
                debug.reason = f"start_reserved:{req.robot_name}"
                return PlannerResult(plans={}, debug=debug)

        # This implementation uses "stay at goal" semantics (agent occupies its goal forever).
        # With this model, shared goal cells among different agents are unsatisfiable.
        seen_goals: Dict[GridCell, str] = {}
        for req in robot_requests:
            if req.goal in seen_goals:
                other = seen_goals[req.goal]
                debug.reason = f"shared_goal_not_supported:{other},{req.robot_name}@{req.goal}"
                return PlannerResult(plans={}, debug=debug)
            seen_goals[req.goal] = req.robot_name

        env = CBSEnvironment(
            robot_requests,
            self.is_free_fn,
            blocked_cells=set(blocked_cells),
            global_vertex_constraints=global_vertex_constraints,
            global_edge_constraints=global_edge_constraints,
            low_level_max_time=ll_max_time,
        )

        open_set: set[HighLevelNode] = set()
        closed_set: set[HighLevelNode] = set()

        start = HighLevelNode()
        start.constraint_dict = {
            agent.robot_name: Constraints() for agent in robot_requests
        }

        env.constraint_dict = start.constraint_dict
        start.solution = env.compute_solution()

        if not start.solution:
            debug.reason = "initial_solution_failed"
            return PlannerResult(plans={}, debug=debug)

        start.cost = env.compute_solution_cost(start.solution)
        open_set.add(start)

        conflicts_resolved = 0
        high_level_nodes = 0

        while open_set:
            elapsed_sec = py_time.monotonic() - planning_start_monotonic
            if elapsed_sec >= planning_time_budget_sec:
                debug.reason = f"planning_timeout:{planning_time_budget_sec:.3f}s"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                return PlannerResult(plans={}, debug=debug)

            if high_level_nodes >= hl_max_nodes:
                debug.reason = f"high_level_node_limit:{hl_max_nodes}"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                return PlannerResult(plans={}, debug=debug)

            current = min(open_set)
            open_set.remove(current)
            closed_set.add(current)
            high_level_nodes += 1

            env.constraint_dict = current.constraint_dict
            conflict = env.get_first_conflict(current.solution)

            if conflict is None:
                plans: Dict[str, RobotPlan] = {}
                total_cells = 0

                for req in robot_requests:
                    state_path = current.solution[req.robot_name]
                    grid_path = [(s.location.x, s.location.y) for s in state_path]
                    total_cells += len(grid_path)
                    plans[req.robot_name] = RobotPlan(
                        robot_name=req.robot_name,
                        start=req.start,
                        goal=req.goal,
                        path=grid_path,
                    )

                debug.reason = "success"
                debug.conflicts_resolved = conflicts_resolved
                debug.high_level_nodes = high_level_nodes
                debug.expanded_nodes = total_cells
                debug.visited_nodes = total_cells
                return PlannerResult(plans=plans, debug=debug)

            conflicts_resolved += 1
            constraint_dict = env.create_constraints_from_conflict(conflict)

            for agent in constraint_dict.keys():
                new_node = deepcopy(current)
                new_node.constraint_dict[agent].add_constraint(constraint_dict[agent])

                env.constraint_dict = new_node.constraint_dict
                new_solution = env.compute_solution()

                if not new_solution:
                    continue

                new_node.solution = new_solution
                new_node.cost = env.compute_solution_cost(new_node.solution)

                if new_node not in closed_set:
                    open_set.add(new_node)

        debug.reason = "no_solution"
        debug.conflicts_resolved = conflicts_resolved
        debug.high_level_nodes = high_level_nodes
        return PlannerResult(plans={}, debug=debug)
