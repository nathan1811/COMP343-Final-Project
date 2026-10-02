"""
models.py
---------
Drone, the Station grid, and the seeded scenario generator.

A seed lays out a whole station: where the airport is, where every drone
starts, how much charge each has, and where the no-fly obstacles / high-drain cells are.

Cell values
    0 FREE      normal tarmac, costs 1x
    1 OBSTACLE  a no-fly obstacle: impassable
    3 GREY      a high-drain / high-drain stretch: passable, costs GREY_COST x
"""
from collections import deque
from dataclasses import dataclass, field, replace
from typing import Tuple, Optional, List, Iterable
import random

Cell = Tuple[int, int]


@dataclass
class Drone:
    id: str
    battery: float          # current battery %, 0-100
    x: int
    y: int
    status: str = "IDLE"    # IDLE / DONE
    destination: Optional[str] = None


@dataclass
class ChargingPad:
    """Display only: drawn on the map, not used by routing or verification."""
    id: str
    type: str           # "Fast" / "Standard"
    power_kw: float
    x: int
    y: int


class Station:
    WIDTH = 10
    HEIGHT = 10
    FREE, OBSTACLE, GREY = 0, 1, 3
    GREY_COST = 3.0                 # a grey cell burns 3x the battery of a normal one
    OBSTACLE_RATIO = 0.14
    GREY_RATIO = 0.07
    STATIC_OBSTACLES = [(3, 3), (3, 4), (3, 5), (6, 2), (6, 3), (7, 7), (8, 7)]

    def __init__(self, seed: Optional[int] = None, reserved: Iterable[Cell] = (),
                 airport: Optional[Cell] = None, supermarket: Optional[Cell] = None):
        self.seed = seed
        self.airport = airport or (self.WIDTH - 1, self.HEIGHT - 1)
        self.supermarket = supermarket or (5, 8)
        self.reserved = set(reserved) | {self.airport, self.supermarket}
        self.grid = [[self.FREE] * self.WIDTH for _ in range(self.HEIGHT)]
        self.dynamic_blocked = set()    # cells blocked by parked drones while planning

        if seed is None:
            for (x, y) in self.STATIC_OBSTACLES:
                if (x, y) not in self.reserved:
                    self.grid[y][x] = self.OBSTACLE
        else:
            self._generate(seed)

    @property
    def landmarks(self):
        """The destinations a drone can be sent to."""
        return {"AIRPORT": self.airport, "SUPERMARKET": self.supermarket}

    # ---------------------------------------------------------------- build
    def _generate(self, seed):
        rng = random.Random(seed)
        placeable = [(x, y) for y in range(self.HEIGHT) for x in range(self.WIDTH)
                     if (x, y) not in self.reserved]

        # Retry with fewer no-fly obstacles until the airport and every start are connected.
        for attempt in range(40):
            relax = max(0.25, 1.0 - attempt * 0.05)
            n_obstacle = int(len(placeable) * self.OBSTACLE_RATIO * relax)
            shuffled = placeable[:]
            rng.shuffle(shuffled)

            self.grid = [[self.FREE] * self.WIDTH for _ in range(self.HEIGHT)]
            for (x, y) in shuffled[:n_obstacle]:
                self.grid[y][x] = self.OBSTACLE

            if self._reserved_all_connected():
                n_grey = max(1, int(len(placeable) * self.GREY_RATIO))
                for (x, y) in shuffled[n_obstacle:n_obstacle + n_grey]:
                    self.grid[y][x] = self.GREY
                return

        # Fallback: empty lot with a few high-drain patches (always solvable).
        self.grid = [[self.FREE] * self.WIDTH for _ in range(self.HEIGHT)]
        shuffled = placeable[:]
        rng.shuffle(shuffled)
        for (x, y) in shuffled[:max(1, int(len(placeable) * self.GREY_RATIO))]:
            self.grid[y][x] = self.GREY

    def _reserved_all_connected(self) -> bool:
        targets = {c for c in self.reserved if self.in_bounds(*c)}
        start = next(iter(targets))
        if self.is_blocked(*start):
            return False
        seen, queue = {start}, deque([start])
        while queue:
            x, y = queue.popleft()
            for n in self.neighbors(x, y):
                if n not in seen:
                    seen.add(n)
                    queue.append(n)
        return targets <= seen

    # -------------------------------------------------------------- queries
    def in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < self.WIDTH and 0 <= y < self.HEIGHT

    def is_blocked(self, x: int, y: int) -> bool:
        return (not self.in_bounds(x, y)
                or (x, y) in self.dynamic_blocked
                or self.grid[y][x] == self.OBSTACLE)

    def is_grey(self, x: int, y: int) -> bool:
        return self.in_bounds(x, y) and self.grid[y][x] == self.GREY

    def cost(self, x: int, y: int) -> float:
        """Battery multiplier for entering this cell."""
        return self.GREY_COST if self.is_grey(x, y) else 1.0

    def neighbors(self, x: int, y: int) -> List[Cell]:
        return [c for c in [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
                if not self.is_blocked(*c)]

    def grey_cells(self) -> List[Cell]:
        return [(x, y) for y in range(self.HEIGHT) for x in range(self.WIDTH)
                if self.grid[y][x] == self.GREY]

    def obstacle_cells(self) -> List[Cell]:
        return [(x, y) for y in range(self.HEIGHT) for x in range(self.WIDTH)
                if self.grid[y][x] == self.OBSTACLE]


# --------------------------------------------------------------------------- #
# Scenarios
# --------------------------------------------------------------------------- #
DEFAULT_DRONES = [
    Drone("DRONE_01", 8, 0, 1),
    Drone("DRONE_02", 35, 1, 0),
    Drone("DRONE_03", 31, 0, 2),
    Drone("DRONE_04", 42, 2, 0),
]
DEFAULT_CHARGING_PADS = [
    ChargingPad("C01", "Fast", 150.0, 8, 1),
    ChargingPad("C02", "Standard", 50.0, 8, 4),
]


@dataclass
class Scenario:
    seed: Optional[int]
    station: Station
    drones: List[Drone] = field(default_factory=list)
    charging_pads: List[ChargingPad] = field(default_factory=list)

    def describe(self) -> str:
        return (f"seed {self.seed}: airport at {self.station.airport}, "
                f"supermarket at {self.station.supermarket}, "
                f"{len(self.drones)} drones, {len(self.charging_pads)} charging_pads, "
                f"{len(self.station.obstacle_cells())} no-fly obstacles, "
                f"{len(self.station.grey_cells())} high-drain cells")


def _spread_pick(rng, pool, count, min_gap):
    """Pick `count` cells at least `min_gap` Manhattan steps apart (loosening if needed)."""
    chosen, candidates, gap = [], pool[:], min_gap
    rng.shuffle(candidates)
    while len(chosen) < count:
        progressed = False
        for c in candidates:
            if c not in chosen and all(abs(c[0] - o[0]) + abs(c[1] - o[1]) >= gap for o in chosen):
                chosen.append(c)
                progressed = True
                if len(chosen) == count:
                    break
        if not progressed:
            if gap <= 1:
                break
            gap -= 1
    return chosen


def generate_scenario(seed: Optional[int] = None, n_drones: int = 4) -> Scenario:
    """seed=None -> the fixed demo station. Otherwise a whole new station from the seed."""
    if seed is None:
        station = Station(reserved=[(v.x, v.y) for v in DEFAULT_DRONES]
                          + [(c.x, c.y) for c in DEFAULT_CHARGING_PADS])
        return Scenario(None, station, [replace(v) for v in DEFAULT_DRONES],
                        [replace(c) for c in DEFAULT_CHARGING_PADS])

    rng = random.Random(seed)
    W, H = Station.WIDTH, Station.HEIGHT
    all_cells = [(x, y) for y in range(H) for x in range(W)]

    border = [c for c in all_cells if c[0] in (0, W - 1) or c[1] in (0, H - 1)]
    airport = rng.choice(border)

    def dist(c):
        return abs(c[0] - airport[0]) + abs(c[1] - airport[1])

    supermarket = rng.choice([c for c in all_cells if dist(c) >= 3])
    taken = {airport, supermarket}

    # Two display-only charging_pads, spread apart.
    charging_pad_cells = _spread_pick(rng, [c for c in all_cells if c not in taken], 2, min_gap=4)
    charging_pads = [ChargingPad("C01", "Fast", 150.0, *charging_pad_cells[0]),
                ChargingPad("C02", "Standard", 50.0, *charging_pad_cells[1])]
    taken |= set(charging_pad_cells)

    pool = [c for c in all_cells if c not in taken and dist(c) >= 4]
    if len(pool) < n_drones:
        pool = [c for c in all_cells if c not in taken]
    cells = _spread_pick(rng, pool, n_drones, min_gap=2)

    drones = [Drone(id=f"DRONE_{i + 1:02d}", battery=round(rng.uniform(10.0, 70.0), 1), x=x, y=y)
                for i, (x, y) in enumerate(cells)]

    station = Station(seed=seed, reserved=cells + charging_pad_cells, airport=airport,
                      supermarket=supermarket)
    return Scenario(seed, station, drones, charging_pads)