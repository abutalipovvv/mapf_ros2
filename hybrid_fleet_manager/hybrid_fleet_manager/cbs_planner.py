from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from heapq import heappop, heappush
from itertools import combinations
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
    ):
        self.agent_requests = agent_requests
        self.is_free_fn = is_free_fn

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

    def state_valid(self, state: State) -> bool:
        if not self.is_free_fn(state.location.x, state.location.y):
            return False
        if VertexConstraint(state.time, state.location) in self.constraints.vertex_constraints:
            return False
        return True

    def transition_valid(self, state_1: State, state_2: State) -> bool:
        edge_constraint = EdgeConstraint(state_1.time, state_1.location, state_2.location)
        return edge_constraint not in self.constraints.edge_constraints

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
            local_solution = self.low_level_search(agent)

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

            if self.is_at_goal(current, agent_name):
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
    def __init__(self, is_free_fn: Callable[[int, int], bool]):
        self.is_free_fn = is_free_fn

    def plan_for_robots(self, robot_requests: List[RobotRequest]) -> PlannerResult:
        debug = PlannerDebug(reason="init")

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

        env = CBSEnvironment(robot_requests, self.is_free_fn)

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