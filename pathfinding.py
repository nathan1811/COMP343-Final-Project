import heapq
import time
from dataclasses import dataclass
from typing import List, Tuple, Set, Dict, Optional

Cell = Tuple[int, int]


@dataclass
class Node:
    """Lab 1 Node, adapted from forklift navigation to Drone cells."""
    x: int
    y: int
    g: float = float("inf")
    h: float = float("inf")
    f: float = float("inf")
    parent: "Node" = None

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        if not isinstance(other, Node):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))


@dataclass
class PathPlan:
    cells: List[Cell]
    cost: float
    grey_cells: int
    nodes_expanded: int = 0
    planning_time: float = 0.0

    @property
    def steps(self) -> int:
        return max(len(self.cells) - 1, 0)


def manhattan_distance(x1: int, y1: int, x2: int, y2: int) -> float:
    return abs(x1 - x2) + abs(y1 - y2)


class DronePathPlanner:
    """Lab 1 ForkliftPlanner adapted to the Drone Station interface."""

    def __init__(self, station):
        self.station = station

    def a_star(self, start: Cell, goal: Cell, heuristic_type: str = "manhattan"):
        start_time = time.time()
        nodes_expanded = 0

        if self.station.is_blocked(*start) or self.station.is_blocked(*goal):
            return [], nodes_expanded, time.time() - start_time

        start_node = Node(start[0], start[1])
        start_node.g = 0
        start_node.h = self._heuristic(start, goal, heuristic_type)
        start_node.f = start_node.g + start_node.h

        open_set = []
        heapq.heappush(open_set, start_node)
        closed_set: Set[Node] = set()
        open_dict: Dict[Tuple[int, int], Node] = {start: start_node}

        while open_set:
            current = heapq.heappop(open_set)

            # Same stale-node protection pattern used by the lab planner.
            known = open_dict.get((current.x, current.y))
            if known is not None and current.g > known.g:
                continue

            if (current.x, current.y) == goal:
                path = []
                while current:
                    path.append((current.x, current.y))
                    current = current.parent
                path.reverse()
                return path, nodes_expanded, time.time() - start_time

            closed_set.add(current)
            nodes_expanded += 1

            for x, y in self.station.neighbors(current.x, current.y):
                if (x, y) in closed_set:
                    continue

                neighbor = Node(x, y)
                tentative_g = current.g + self.station.cost(x, y)

                if ((x, y) not in open_dict) or (tentative_g < open_dict[(x, y)].g):
                    if (x, y) in open_dict:
                        neighbor = open_dict[(x, y)]
                    neighbor.parent = current
                    neighbor.g = tentative_g
                    neighbor.h = self._heuristic((x, y), goal, heuristic_type)
                    neighbor.f = neighbor.g + neighbor.h
                    heapq.heappush(open_set, neighbor)
                    open_dict[(x, y)] = neighbor

        return [], nodes_expanded, time.time() - start_time

    def _heuristic(self, cell: Cell, goal: Cell, heuristic_type: str) -> float:
        x, y = cell
        gx, gy = goal
        if heuristic_type == "manhattan":
            return manhattan_distance(x, y, gx, gy)
        if heuristic_type == "euclidean":
            return ((x - gx) ** 2 + (y - gy) ** 2) ** 0.5
        if heuristic_type == "chebyshev":
            return max(abs(x - gx), abs(y - gy))
        raise ValueError("Invalid heuristic")


def plan_path(station, start: Cell, goal: Cell) -> Optional[PathPlan]:
    """Public wrapper retaining the original project API."""
    planner = DronePathPlanner(station)
    cells, nodes_expanded, planning_time = planner.a_star(start, goal, "manhattan")
    if not cells:
        return None
    grey = sum(station.is_grey(*c) for c in cells[1:])
    cost = sum(station.cost(*c) for c in cells[1:])
    return PathPlan(cells, cost, grey, nodes_expanded, planning_time)
