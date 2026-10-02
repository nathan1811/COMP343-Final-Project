"""
orchestrator.py
---------------
StationManager wires the pipeline together:

    command -> LLM parser -> A* route -> optional charging stop -> Z3 verifier
            -> (rejected: drone stays put) or (verified: drone delivers) -> DB + log

Destinations: AIRPORT or SUPERMARKET.
Each A* movement step takes 15 minutes. Charging time depends on pad power
and the requested target battery percentage.
"""
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Dict, Any, List, Optional, Tuple
import re
import time
import math
import random

import database as db
import llm_parser
import pathfinding
import verifier
from models import Scenario, Drone, ChargingPad, generate_scenario
from perception import DroneStateEstimator
from adaptive_pilot import AdaptivePilot


STEP_MINUTES = 15.0
BATTERY_CAPACITY_KWH = 100.0
RESERVE_BATTERY_PCT = 15.0

# Each normal movement step has a random battery drain between these bounds.
# The planner/verifier use the upper bound so a route is safe even in the
# worst possible draw. The actual animation/execution uses a fresh random
# value for every movement step.
MIN_STEP_DRAIN_PCT = 1.5
MAX_STEP_DRAIN_PCT = 3.0
AVERAGE_STEP_DRAIN_PCT = (MIN_STEP_DRAIN_PCT + MAX_STEP_DRAIN_PCT) / 2.0
PLANNING_STEP_DRAIN_PCT = MAX_STEP_DRAIN_PCT


@dataclass
class Trip:
    drone_id: str
    drain_per_cell: float
    destination: str = "AIRPORT"
    log: List[str] = field(default_factory=list)
    plan: Any = None
    route_cells: Any = None          # exact full route shown on the dashboard
    report: Any = None
    task_id: Optional[int] = None
    start_battery: float = 0.0
    driven: bool = False
    react_trace: Any = None
    current_minutes: float = 0.0
    deadline_minutes: Optional[float] = None
    deadline_text: Optional[str] = None
    arrival_minutes: Optional[float] = None
    charging_pad: Optional[ChargingPad] = None
    charge_to_pct: Optional[float] = None
    charge_minutes: float = 0.0
    execution_cells: Optional[List[Tuple[int, int]]] = None
    step_drains: List[float] = field(default_factory=list)
    actual_final_battery: Optional[float] = None
    pilot_decisions: List[Any] = field(default_factory=list)

    @property
    def verified(self) -> bool:
        return self.report is not None and self.report.plan_ok


@dataclass
class MultiTrip:
    trips: List[Trip] = field(default_factory=list)
    drain_per_cell: float = 2.0
    current_minutes: float = 0.0

    @property
    def verified(self) -> bool:
        return bool(self.trips) and all(t.verified for t in self.trips)


class StationManager:
    def __init__(self, db_path: str = db.DB_PATH, reset: bool = True, seed: Optional[int] = None):
        self.db_path = db_path
        db.init_db(db_path, reset=reset)
        self.seed = seed
        self._install(generate_scenario(seed))

    def _install(self, scenario: Scenario):
        self.scenario = scenario
        self.station = scenario.station
        self.estimators = {d.id: DroneStateEstimator(d.battery, d.x, d.y)
                           for d in scenario.drones}
        # Course-derived adaptive pilot. It is advisory; A* and Z3 remain
        # authoritative for routing and safety.
        self.adaptive_pilot = AdaptivePilot(seed=self.seed or 0)
        db.seed(scenario.drones, self.db_path)

    def load_scenario(self, seed: Optional[int]) -> Scenario:
        db.init_db(self.db_path, reset=True)
        self.seed = seed
        self._install(generate_scenario(seed))
        db.log_event("MAP_LOADED", self.scenario.describe(), self.db_path)
        return self.scenario

    def get_drones(self) -> List[Dict[str, Any]]:
        return db.fetch_all("drones", self.db_path)

    def get_tasks(self):
        return db.fetch_all("tasks", self.db_path)

    def get_events(self, limit=40):
        return db.recent_events(limit, self.db_path)

    def perception_snapshot(self) -> Dict[str, Any]:
        return {r["id"]: self.estimators[r["id"]].tick(r["battery"], r["x"], r["y"])
                for r in self.get_drones()}

    def _drone(self, drone_id: str) -> Optional[Drone]:
        row = next((r for r in self.get_drones() if r["id"] == drone_id), None)
        return Drone(**row) if row else None

    @contextmanager
    def _park_others(self, drone_id: str):
        cells = {(r["x"], r["y"]) for r in self.get_drones() if r["id"] != drone_id}
        cells -= set(self.station.landmarks.values())
        previous = self.station.dynamic_blocked
        self.station.dynamic_blocked = cells
        try:
            yield
        finally:
            self.station.dynamic_blocked = previous

    @staticmethod
    def _clock_to_minutes(value: Optional[str]) -> Optional[float]:
        if not value:
            return None
        text = str(value).strip().lower().replace(" ", "")
        m = re.fullmatch(r"(\d{1,2})(?::(\d{2}))?(am|pm)?", text)
        if not m:
            return None
        hour = int(m.group(1))
        minute = int(m.group(2) or 0)
        suffix = m.group(3)
        if minute >= 60:
            return None
        if suffix:
            if hour < 1 or hour > 12:
                return None
            if suffix == "am":
                hour = 0 if hour == 12 else hour
            else:
                hour = 12 if hour == 12 else hour + 12
        elif hour > 23:
            return None
        return hour * 60 + minute

    @staticmethod
    def _deadline_absolute(current_minutes: float, deadline_clock: Optional[float]) -> Optional[float]:
        if deadline_clock is None:
            return None
        deadline = deadline_clock
        if deadline <= current_minutes:
            deadline += 24 * 60
        return deadline

    @staticmethod
    def _combine_plans(first, second):
        if first is None or second is None:
            return None
        from pathfinding import PathPlan
        cells = first.cells + second.cells[1:]
        return PathPlan(
            cells=cells,
            cost=first.cost + second.cost,
            grey_cells=first.grey_cells + second.grey_cells,
            nodes_expanded=first.nodes_expanded + second.nodes_expanded,
            planning_time=first.planning_time + second.planning_time,
        )

    @staticmethod
    def _charging_minutes(battery_before: float, target: float, power_kw: float) -> float:
        delta_pct = max(0.0, target - battery_before)
        if delta_pct <= 0 or power_kw <= 0:
            return 0.0
        energy_kwh = (delta_pct / 100.0) * BATTERY_CAPACITY_KWH
        return energy_kwh / power_kw * 60.0

    @staticmethod
    def _random_step_drain() -> float:
        return random.uniform(MIN_STEP_DRAIN_PCT, MAX_STEP_DRAIN_PCT)

    def _build_actual_consumption(self, trip: Trip) -> float:
        """Generate the actual random drain for every movement step.

        Planning remains conservative: all routing, charging and SMT checks use
        the 3.0% upper bound. Execution uses the actual random 1.5%-3.0% draw for
        each movement step, including the extra 3x multiplier on grey cells.
        """
        if not trip.plan or not trip.plan.cells:
            trip.step_drains = []
            trip.actual_final_battery = trip.start_battery
            return trip.start_battery

        drains = [self._random_step_drain() for _ in trip.plan.cells[1:]]
        trip.step_drains = drains

        battery = trip.start_battery
        pad_cell = ((trip.charging_pad.x, trip.charging_pad.y)
                    if trip.charging_pad is not None else None)
        drain_index = 0
        charged = False

        for cell in trip.plan.cells[1:]:
            actual_drain = drains[drain_index]
            drain_index += 1
            battery -= actual_drain * self.station.cost(*cell)

            if pad_cell is not None and tuple(cell) == pad_cell and not charged:
                battery = trip.charge_to_pct if trip.charge_to_pct is not None else battery
                charged = True

        trip.actual_final_battery = max(0.0, battery)
        return trip.actual_final_battery

    def _choose_charger(self, drone: Drone, destination: str, drain_per_cell: float,
                        current_minutes: float, deadline_minutes: Optional[float],
                        requested_charge_to: Optional[float] = None) -> Optional[Dict[str, Any]]:
        goal = self.station.landmarks[destination]
        candidates = []

        for pad in self.scenario.charging_pads:
                to_pad = pathfinding.plan_path(self.station, (drone.x, drone.y), (pad.x, pad.y))
                if to_pad is None:
                    continue

                battery_at_pad = drone.battery - to_pad.cost * drain_per_cell
                if battery_at_pad < 0:
                    continue

                to_destination = pathfinding.plan_path(self.station, (pad.x, pad.y), goal)
                if to_destination is None:
                    continue

                destination_energy = to_destination.cost * drain_per_cell

                # Charging is fully automatic. The drone should arrive at the
                # final destination with at least a 15% battery reserve.
                # Therefore the required charging target is: energy needed for
                # the final leg + 15% reserve. The operator no longer needs to
                # specify a charging percentage.
                required_target = destination_energy + RESERVE_BATTERY_PCT
                if required_target > 100.0 + 1e-9:
                    continue

                target = max(battery_at_pad, required_target)
                if target > 100.0:
                    continue

                charge_minutes = self._charging_minutes(battery_at_pad, target, pad.power_kw)
                travel_minutes = (to_pad.steps + to_destination.steps) * STEP_MINUTES
                arrival = current_minutes + travel_minutes + charge_minutes
                deadline_ok = deadline_minutes is None or arrival <= deadline_minutes

                # Primary criterion: earliest arrival. Ties favour the shorter
                # route to the pad, then the higher-power pad.
                score = (arrival, to_pad.steps, -pad.power_kw)
                candidates.append({
                    "pad": pad,
                    "to_pad_plan": to_pad,
                    "to_destination_plan": to_destination,
                    "battery_at_pad": battery_at_pad,
                    "charge_to_pct": target,
                    "charge_needed": max(0.0, target - battery_at_pad),
                    "charge_minutes": charge_minutes,
                    "arrival_minutes": arrival,
                    "deadline_ok": deadline_ok,
                    "score": score,
                })

        if not candidates:
            return None
        candidates.sort(key=lambda c: c["score"])
        return candidates[0]

    # ------------------------------------------------------- multi-drone planner
    @staticmethod
    def _reserve_path(reservations, cells):
        """Reserve exactly the tiles occupied at each simulation tick.

        Finished drones are not reserved forever: once a delivery is complete,
        another drone may use that destination tile later.
        """
        for tick, cell in enumerate(cells):
            reservations.setdefault(tick, set()).add(tuple(cell))

    def _verify_multi_schedule(self, scheduled_trips, candidate_cells):
        schedules = {t.drone_id: list(t.execution_cells or []) for t in scheduled_trips}
        schedules["__candidate__"] = list(candidate_cells)
        charger_cells = {p.id: (p.x, p.y) for p in self.scenario.charging_pads}
        return verifier.verify_multi_drone_schedule_smt(schedules, charger_cells)

    def _schedule_fixed_route(self, cells, reservations, charger_cell=None,
                              charge_ticks=0, start_tick=0, max_wait=300):
        """Schedule a fixed A* route by inserting WAITs only.

        ``cells`` is a spatial A* route. ``start_tick`` is the absolute
        simulation tick at which ``cells[0]`` is occupied. This is important
        when a route has a charging stop: the second A* leg must be scheduled
        from the *end* of the charging interval, not from tick zero.

        Routes are allowed to overlap. A drone waits in its current tile only
        when its next tile would be occupied at the same tick. If the next tile
        is a charger, the whole charging interval must also be free before the
        drone enters it.
        """
        cells = [tuple(c) for c in cells]
        if not cells:
            return None

        timed = [cells[0]]
        current = cells[0]
        tick = int(start_tick)

        for next_cell in cells[1:]:
            waits = 0
            while True:
                next_tick = tick + 1
                blocked = next_cell in reservations.get(next_tick, set())

                # Adaptive pilot: when the next cell is already reserved,
                # learn/choose a cautious wait before attempting the move.
                # Conflict resolution still owns the actual reservation rule.
                terrain = "GREY" if self.station.is_grey(*next_cell) else "NORMAL"
                pilot_decision = self.adaptive_pilot.choose_action(
                    terrain, blocked, 50.0, epsilon=0.0
                )
                if (not blocked and terrain == "GREY" and
                        pilot_decision.action == "CAUTIOUS_WAIT"):
                    # The adaptive pilot can add one cautious 15-minute hold
                    # before entering an adverse/high-drain cell. This is a
                    # timing adjustment only; Z3 still verifies the schedule.
                    timed.append(current)
                    tick += 1
                    waits += 1
                    if waits > max_wait:
                        return None
                    next_tick = tick + 1

                if charger_cell is not None and next_cell == charger_cell and charge_ticks:
                    blocked = blocked or any(
                        charger_cell in reservations.get(next_tick + j, set())
                        for j in range(charge_ticks)
                    )

                if not blocked:
                    break

                timed.append(current)
                tick += 1
                waits += 1
                if waits > max_wait:
                    return None

            tick += 1
            timed.append(next_cell)
            current = next_cell

            if charger_cell is not None and next_cell == charger_cell and charge_ticks:
                for _ in range(charge_ticks):
                    tick += 1
                    timed.append(charger_cell)

        return timed

    def _movement_only_plan(self, timed_cells):
        """Remove WAIT repetitions so battery is charged only for movement."""
        from pathfinding import PathPlan
        cells = []
        for cell in timed_cells:
            cell = tuple(cell)
            if not cells or cell != cells[-1]:
                cells.append(cell)
        return PathPlan(
            cells=cells,
            cost=sum(self.station.cost(*c) for c in cells[1:]),
            grey_cells=sum(self.station.is_grey(*c) for c in cells[1:]),
        )

    def _multi_charger(self, drone, destination, drain_per_cell, current_minutes,
                       reservations):
        """Choose a charger using an independent A* route.

        Other drones affect timing only; they never alter the spatial route.
        """
        goal = self.station.landmarks[destination]
        candidates = []
        for pad in self.scenario.charging_pads:
            to_pad = pathfinding.plan_path(self.station, (drone.x, drone.y), (pad.x, pad.y))
            to_destination = pathfinding.plan_path(self.station, (pad.x, pad.y), goal)
            if to_pad is None or to_destination is None:
                continue

            battery_at_pad = drone.battery - to_pad.cost * drain_per_cell
            if battery_at_pad < -1e-9:
                continue
            destination_energy = to_destination.cost * drain_per_cell
            required_target = destination_energy + RESERVE_BATTERY_PCT
            if required_target > 100.0 + 1e-9:
                continue
            target = max(battery_at_pad, required_target)
            charge_minutes = self._charging_minutes(battery_at_pad, target, pad.power_kw)
            charge_ticks = max(0, math.ceil(charge_minutes / STEP_MINUTES - 1e-9))
            candidates.append({
                'pad': pad,
                'to_pad_plan': to_pad,
                'to_destination_plan': to_destination,
                'battery_at_pad': battery_at_pad,
                'charge_to_pct': target,
                'charge_needed': max(0.0, target - battery_at_pad),
                'charge_minutes': charge_minutes,
                'charge_ticks': charge_ticks,
                'arrival_minutes': current_minutes +
                    (to_pad.steps + to_destination.steps) * STEP_MINUTES + charge_minutes,
                'score': (current_minutes +
                          (to_pad.steps + to_destination.steps) * STEP_MINUTES + charge_minutes,
                          to_pad.steps, -pad.power_kw),
            })
        return min(candidates, key=lambda c: c['score']) if candidates else None

    def plan_multi(self, jobs: List[Dict[str, Any]], current_minutes: float = 0.0) -> MultiTrip:
        """Plan multiple drones on one timeline.

        Each drone first gets its own ordinary A* spatial route. Routes are not
        blocked by one another. Drones are then scheduled in ascending starting
        battery order. If the next tile is occupied at the same tick, the
        higher-battery drone waits in its previous tile until it is free.
        Z3 verifies the complete resulting timeline before execution.
        """
        if not jobs:
            return MultiTrip([], 2.0, current_minutes)

        # Operator-supplied drain values are retained only for backwards compatibility.
        # The physical model is now random 1.5%-3.0% per movement step, and planning
        # uses the conservative 3% worst case.
        drain = PLANNING_STEP_DRAIN_PCT
        indexed = list(enumerate(jobs))
        indexed.sort(key=lambda item: (
            self._drone(item[1]['drone_id']).battery
            if self._drone(item[1]['drone_id']) else float('inf'),
            item[0],
        ))
        jobs = [job for _, job in indexed]

        priority_text = ', '.join(
            f"{job['drone_id']} ({self._drone(job['drone_id']).battery:.1f}%)"
            for job in jobs if self._drone(job['drone_id'])
        )
        reservations: Dict[int, set] = {}
        trips: List[Trip] = []

        for job in jobs:
            drone_id = job['drone_id']
            destination = str(job['destination']).upper()
            deadline_text = job.get('deadline_text')
            deadline_clock = self._clock_to_minutes(deadline_text)
            deadline_abs = self._deadline_absolute(current_minutes, deadline_clock)
            drone = self._drone(drone_id)

            trip = Trip(
                drone_id, drain, destination,
                log=[
                    f"Multi-drone command: {drone_id} -> {destination}",
                    f"Priority order (lowest starting battery first): {priority_text}",
                ],
                current_minutes=current_minutes,
                deadline_minutes=deadline_abs,
                deadline_text=deadline_text,
            )

            if drone is None or destination not in self.station.landmarks:
                trip.report = verifier.VerificationReport(plan_ok=False)
                trip.report.add('Multi-drone schedule', False, 'invalid drone or destination')
                trips.append(trip)
                continue

            trip.start_battery = drone.battery
            trip.task_id = db.create_task(
                f"MULTI: deliver {drone_id} to {destination} [{drain:g}% per cell]",
                drone_id, self.db_path
            )
            goal = self.station.landmarks[destination]

            # Spatial planning is independent for every drone. Other drones do
            # not become obstacles: only the later time schedule can make one
            # drone wait.
            direct = pathfinding.plan_path(self.station, (drone.x, drone.y), goal)
            direct_energy = direct.cost * drain if direct else float('inf')
            if direct is not None:
                pilot_preview = self.adaptive_pilot.route_advice(
                    direct.cells, self.station, drone.battery
                )
                cautious = sum(1 for d in pilot_preview if d.action == "CAUTIOUS_WAIT")
                trip.log.append(
                    f"Adaptive Pilot: evaluated {len(pilot_preview)} movement steps; "
                    f"{cautious} step(s) flagged for cautious handling. "
                    f"Q-learning policy is advisory; A* and Z3 remain authoritative."
                )
            chosen = None

            if direct is not None and direct_energy <= drone.battery + 1e-9:
                chosen = ('direct', direct)
            elif direct is not None:
                charging = self._multi_charger(
                    drone, destination, drain, current_minutes, reservations
                )
                if charging:
                    chosen = ('charging', charging)

            if chosen is None:
                trip.report = verifier.VerificationReport(plan_ok=False)
                trip.report.add(
                    'Multi-drone schedule', False,
                    'no physical route or viable charging plan'
                )
                db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                trip.log.append(f"{drone_id} cannot be scheduled; drone does not move.")
                trips.append(trip)
                continue

            kind, data = chosen
            if kind == 'direct':
                timed = self._schedule_fixed_route(data.cells, reservations)
                charging = None
                if timed is None:
                    trip.report = verifier.VerificationReport(plan_ok=False)
                    trip.report.add('Multi-drone schedule', False, 'could not create a safe waiting schedule')
                    db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                    trips.append(trip)
                    continue
                trip.execution_cells = timed
                trip.route_cells = list(data.cells)
                trip.plan = self._movement_only_plan(timed)
                trip.arrival_minutes = current_minutes + (len(timed) - 1) * STEP_MINUTES

            else:
                charging = data
                pad_cell = (charging['pad'].x, charging['pad'].y)
                outbound = charging['to_pad_plan'].cells
                final_leg = charging['to_destination_plan'].cells

                # Wait before entering the charger if any part of the charging
                # interval overlaps an already reserved charger occupancy.
                timed_outbound = self._schedule_fixed_route(
                    outbound, reservations,
                    charger_cell=pad_cell,
                    charge_ticks=charging['charge_ticks'],
                )
                if timed_outbound is None:
                    trip.report = verifier.VerificationReport(plan_ok=False)
                    trip.report.add('Multi-drone schedule', False, 'could not schedule charger access safely')
                    db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                    trips.append(trip)
                    continue

                # The charger occupancy is already in timed_outbound. The
                # final A* leg MUST continue from the absolute tick at which
                # charging ends. The old implementation restarted this leg at
                # tick 0, which could create false Z3 collisions or incorrect
                # waits later in the route.
                timed = list(timed_outbound)
                outbound_end_tick = len(timed_outbound) - 1
                final_route = [pad_cell] + list(final_leg[1:])
                timed_final = self._schedule_fixed_route(
                    final_route, reservations, start_tick=outbound_end_tick
                )
                if timed_final is None:
                    trip.report = verifier.VerificationReport(plan_ok=False)
                    trip.report.add(
                        'Multi-drone schedule', False,
                        'could not schedule the final leg after charging'
                    )
                    db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                    trips.append(trip)
                    continue

                # timed_final starts with the same charger cell that already
                # ends timed_outbound, so avoid duplicating that single tick.
                timed.extend(timed_final[1:])

                trip.charging_pad = charging['pad']
                trip.charge_to_pct = charging['charge_to_pct']
                trip.charge_minutes = charging['charge_minutes']
                trip.execution_cells = timed
                trip.route_cells = list(outbound) + list(final_leg[1:])
                trip.plan = self._movement_only_plan(timed)

                # Movement and waiting consume 15-minute simulation ticks.
                # Actual charging time is kept at the calculated minute value.
                movement_steps = (len(outbound) - 1) + (len(final_leg) - 1)
                wait_ticks = max(0,
                    (len(timed) - 1) - movement_steps - charging['charge_ticks']
                )
                trip.arrival_minutes = (
                    current_minutes
                    + (movement_steps + wait_ticks) * STEP_MINUTES
                    + charging['charge_minutes']
                )

            trip.report = verifier.verify_trip(
                drone, trip.plan, self.station, drain, destination,
                charging_plan=data if kind == 'charging' else None,
                deadline_ok=(deadline_abs is None or trip.arrival_minutes <= deadline_abs),
                arrival_minutes=trip.arrival_minutes,
                deadline_text=deadline_text,
            )

            if trip.report.plan_ok:
                self._build_actual_consumption(trip)

            # Z3 is the final formal gate over the complete timeline.
            previous = [t for t in trips if t.verified and t.execution_cells]
            z3_result = self._verify_multi_schedule(previous, trip.execution_cells or [])

            if trip.verified and z3_result['safe']:
                self._reserve_path(reservations, trip.execution_cells or [])
                db.update_task_status(trip.task_id, 'VERIFIED', self.db_path)
                trip.log.append(
                    f"Z3 formally verified the multi-drone schedule. "
                    f"Estimated arrival: {self._format_clock(trip.arrival_minutes)}."
                )
            elif trip.verified:
                trip.report = verifier.VerificationReport(plan_ok=False)
                trip.report.add(
                    'Multi-drone Z3 safety', False,
                    'Z3 found two drones occupying the same tile at the same time.'
                )
                db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                trip.log.append("Z3 rejected the timeline; drone does not move.")
            else:
                db.update_task_status(trip.task_id, 'REJECTED', self.db_path)
                trip.log.append("Drone does not move because its battery/deadline check failed.")

            trips.append(trip)

        return MultiTrip(trips=trips, drain_per_cell=drain, current_minutes=current_minutes)

    def deliver_multi(self, multi: MultiTrip) -> MultiTrip:
        """Commit every verified drone after the synchronized multi-drone animation."""
        for trip in multi.trips:
            if trip.verified and not trip.driven:
                self.deliver(trip)
        return multi

    # ------------------------------------------------------------- pipeline
    def plan_from_command(self, command: str, current_minutes: float = 0.0):
        """Parse and plan either one drone or multiple drones from one natural-language command."""
        ids = [r["id"] for r in self.get_drones()]
        task = llm_parser.parse_command(command, ids)

        if task.get("multi"):
            # The command may still contain an old fixed drain value, but the
            # drone model now uses a random 1.5%-3.0% draw per movement step.
            # Planning uses the 3% worst case for safety.
            drain = PLANNING_STEP_DRAIN_PCT
            jobs = []
            for item in task.get("trips", []):
                jobs.append({
                    "drone_id": item.get("drone_id"),
                    "destination": item.get("destination", "AIRPORT"),
                    "drain_per_cell": drain,
                    "deadline_text": item.get("deadline"),
                })
            multi = self.plan_multi(jobs, current_minutes=current_minutes)
            for t in multi.trips:
                t.log.insert(0, f'Operator command received: "{command}"')
                t.log.append(
                    f"Natural-language multi-drone assignment: {t.drone_id} -> {t.destination}; "
                    f"actual battery use=random {MIN_STEP_DRAIN_PCT:g}-{MAX_STEP_DRAIN_PCT:g}% per cell "
                    f"(planning safety bound={PLANNING_STEP_DRAIN_PCT:g}%)."
                )
            db.log_event(
                "MULTI_TASK_PARSED",
                f"{len(jobs)} drones parsed from natural language; actual drain is random "
                f"{MIN_STEP_DRAIN_PCT:g}-{MAX_STEP_DRAIN_PCT:g}% per movement step; "
                f"planning uses {PLANNING_STEP_DRAIN_PCT:g}% worst case.",
                self.db_path,
            )
            return multi

        log = [f"Operator command received: \"{command}\""]
        if task["drone_id"] is None:
            unresolved = task.get("_unresolved_drone_id", "<none named>")
            log.append(f"ERROR: drone '{unresolved}' does not exist. "
                       f"Known drones: {', '.join(ids)}. Nothing will move.")
            db.log_event("TASK_PARSE_FAILED", log[-1], self.db_path)
            return Trip(unresolved, task["drain_per_cell"],
                        task["destination"], log, current_minutes=current_minutes)

        deadline_text = task.get("deadline")
        log.append(f"LLM parser ({task['_source']}) -> drone={task['drone_id']}, "
                   f"destination={task['destination']}, battery use=random "
                   f"{MIN_STEP_DRAIN_PCT:g}-{MAX_STEP_DRAIN_PCT:g}% per movement step "
                   f"(planning safety bound={PLANNING_STEP_DRAIN_PCT:g}%)")
        if deadline_text:
            log.append(f"Deadline requested: {deadline_text}")
        db.log_event("TASK_PARSED", log[-1], self.db_path)
        return self.plan_trip(task["drone_id"], task["drain_per_cell"], task["destination"],
                              log, command, current_minutes, deadline_text, None)

    def plan_trip(self, drone_id: str, drain_per_cell: float = 2.0,
                  destination: str = "AIRPORT", log: Optional[List[str]] = None,
                  command: Optional[str] = None, current_minutes: float = 0.0,
                  deadline_text: Optional[str] = None,
                  charge_to_pct: Optional[float] = None) -> Trip:
        log = log if log is not None else []
        destination = destination.upper()
        # The input parameter is kept for API compatibility, but the physical
        # model is now random 1.5%-3.0% per movement step. Use the worst case for
        # route/charger selection and formal verification.
        drain_per_cell = PLANNING_STEP_DRAIN_PCT
        deadline_clock = self._clock_to_minutes(deadline_text)
        deadline_abs = self._deadline_absolute(current_minutes, deadline_clock)
        trip = Trip(drone_id, drain_per_cell, destination, log,
                    current_minutes=current_minutes, deadline_minutes=deadline_abs,
                    deadline_text=deadline_text)

        drone = self._drone(drone_id)
        if drone is None:
            log.append(f"ERROR: unknown drone '{drone_id}'.")
            return trip
        if destination not in self.station.landmarks:
            log.append(f"ERROR: unknown destination '{destination}'.")
            return trip
        trip.start_battery = drone.battery

        command = command or (f"MANUAL: deliver {drone_id} to {destination} "
                              f"[{drain_per_cell:g}% per cell]")
        trip.task_id = db.create_task(command, drone_id, self.db_path)
        goal = self.station.landmarks[destination]

        # First try the original direct route. Charging is only introduced when
        # the drone cannot complete that route on its current battery (or no
        # direct route exists).
        with self._park_others(drone_id):
            direct_plan = pathfinding.plan_path(self.station, (drone.x, drone.y), goal)

        direct_energy = direct_plan.cost * drain_per_cell if direct_plan else float("inf")

        # IMPORTANT: charging is a battery fallback, not an alternative route.
        # If A* can find a direct route and the drone has enough battery for it,
        # always deliver directly. If A* cannot find any route to the destination,
        # do NOT send the drone to a charger merely because the destination route
        # is unavailable; charging cannot fix a blocked/no-route problem.
        use_charging = direct_plan is not None and direct_energy > drone.battery + 1e-9

        if direct_plan is not None and not use_charging:
            trip.plan = direct_plan
            trip.route_cells = list(direct_plan.cells)
            trip.arrival_minutes = current_minutes + direct_plan.steps * STEP_MINUTES
            log.append(f"A* direct route to {destination}: "
                       f"{' -> '.join(map(str, direct_plan.cells))}")
            deadline_ok = deadline_abs is None or trip.arrival_minutes <= deadline_abs
            trip.report = verifier.verify_trip(
                drone, direct_plan, self.station, drain_per_cell, destination,
                deadline_ok=deadline_ok if deadline_abs is not None else None,
                arrival_minutes=trip.arrival_minutes,
                deadline_text=deadline_text,
            )
        else:
            # At this point direct_plan is None: there is no physical A* route
            # to the destination. Do not invent a charging stop.
            if direct_plan is None:
                log.append("No direct A* route to the destination; charging cannot fix a blocked route.")
                trip.plan = None
                trip.report = verifier.VerificationReport(plan_ok=False)
                trip.report.add(
                    "Route exists", False,
                    f"no route to the {destination.lower()} (blocked by obstacles or parked drones)"
                )
            else:
                log.append(f"Direct route needs {direct_energy:.1f}% but drone has "
                           f"{drone.battery:.1f}%; charging stop required.")

            charging = None if direct_plan is None else self._choose_charger(
                drone, destination, drain_per_cell, current_minutes,
                deadline_abs, charge_to_pct
            )
            if charging is None:
                # A direct physical route exists, but no charging pad can make
                # the delivery feasible. Keep the direct destination route only
                # as an attempted/dotted path. Never show a charger as the target
                # and never move the drone there.
                trip.plan = direct_plan
                trip.route_cells = list(direct_plan.cells)
                trip.arrival_minutes = current_minutes + direct_plan.steps * STEP_MINUTES
                log.append("No viable charging pad can complete the delivery; "
                           "showing the attempted direct route as a dotted path.")
                trip.report = verifier.verify_trip(
                    drone, direct_plan, self.station, drain_per_cell, destination,
                    deadline_ok=False if deadline_abs is not None and trip.arrival_minutes > deadline_abs else None,
                    arrival_minutes=trip.arrival_minutes,
                    deadline_text=deadline_text,
                )
            else:
                trip.charging_pad = charging["pad"]
                trip.charge_to_pct = charging["charge_to_pct"]
                trip.charge_minutes = charging["charge_minutes"]
                trip.arrival_minutes = charging["arrival_minutes"]
                trip.plan = self._combine_plans(charging["to_pad_plan"], charging["to_destination_plan"])
                trip.route_cells = list(trip.plan.cells) if trip.plan is not None else []
                trip.log.append(
                    f"Charging decision: {trip.charging_pad.id} ({trip.charging_pad.power_kw:g} kW); "
                    f"arrive pad with {charging['battery_at_pad']:.1f}%, charge to "
                    f"{trip.charge_to_pct:.1f}% in {trip.charge_minutes:.1f} min; "
                    f"estimated arrival {self._format_clock(trip.arrival_minutes)}."
                )
                trip.log.append(f"A* route via {trip.charging_pad.id}: "
                                f"{' -> '.join(map(str, trip.plan.cells))}")
                trip.report = verifier.verify_trip(
                    drone, trip.plan, self.station, drain_per_cell, destination,
                    charging_plan=charging,
                    deadline_ok=charging["deadline_ok"] if deadline_abs is not None else None,
                    arrival_minutes=trip.arrival_minutes,
                    deadline_text=deadline_text,
                )

        # Adaptive pilot is advisory for the single-drone route. It learns local
        # movement preferences from terrain conditions without replacing A*.
        if trip.plan is not None and trip.plan.cells:
            pilot_preview = self.adaptive_pilot.route_advice(
                trip.plan.cells, self.station, trip.start_battery
            )
            trip.pilot_decisions = pilot_preview
            cautious = sum(1 for d in pilot_preview if d.action == "CAUTIOUS_WAIT")
            log.append(
                f"Adaptive Pilot: evaluated {len(pilot_preview)} movement steps; "
                f"{cautious} step(s) flagged for cautious handling. "
                f"A* route remains authoritative and Z3 remains the final safety gate."
            )

        # Generate the actual per-step random consumption only after the
        # conservative route/charger/SMT plan has been established.
        if trip.verified if trip.report is not None else False:
            self._build_actual_consumption(trip)
            if trip.actual_final_battery is not None:
                trip.log.append(
                    f"Actual random consumption: {MIN_STEP_DRAIN_PCT:g}-{MAX_STEP_DRAIN_PCT:g}% "
                    f"per movement step; simulated final battery={trip.actual_final_battery:.1f}%."
                )

        if llm_parser.LAST_TRACE:
            trip.react_trace = list(llm_parser.LAST_TRACE)
        log.extend(trip.report.as_lines())
        db.log_event("VERIFICATION_PASSED" if trip.verified else "VERIFICATION_FAILED",
                     "; ".join(trip.report.as_lines()), self.db_path)

        if trip.verified:
            db.update_task_status(trip.task_id, "VERIFIED", self.db_path)
        else:
            if (trip.deadline_minutes is not None and
                    trip.arrival_minutes is not None and
                    trip.arrival_minutes > trip.deadline_minutes):
                log.append(
                    f"Cannot reach the final destination in time: estimated arrival "
                    f"{self._format_clock(trip.arrival_minutes)} exceeds the "
                    f"{trip.deadline_text} deadline. Drone does not move."
                )
            else:
                log.append(
                    f"{drone_id} cannot complete the delivery to "
                    f"{destination.lower()}. Drone does not move."
                )
            db.update_task_status(trip.task_id, "REJECTED", self.db_path)
        return trip

    @staticmethod
    def _format_clock(minutes: float) -> str:
        minutes = minutes % (24 * 60)
        h = int(minutes // 60)
        m = int(round(minutes % 60))
        if m == 60:
            h = (h + 1) % 24
            m = 0
        suffix = "AM" if h < 12 else "PM"
        display_h = h % 12 or 12
        return f"{display_h}:{m:02d} {suffix}"

    def get_delivery_animation(self, trip: Trip):
        """Return step-by-step execution data for the dashboard animation."""
        if not trip.verified or not trip.plan:
            return []

        steps = []
        battery = trip.start_battery
        pad_cell = ((trip.charging_pad.x, trip.charging_pad.y)
                    if trip.charging_pad is not None else None)

        if not trip.step_drains or len(trip.step_drains) != len(trip.plan.cells) - 1:
            self._build_actual_consumption(trip)

        drain_index = 0
        for i, cell in enumerate(trip.plan.cells):
            if i == 0:
                steps.append({
                    "cell": cell,
                    "battery": battery,
                    "charging": False,
                    "charge_minutes": 0.0,
                })
                continue

            actual_drain = trip.step_drains[drain_index]
            drain_index += 1
            battery_before_move = battery
            battery -= self.station.cost(*cell) * actual_drain
            battery = max(0.0, battery)

            item = {
                "cell": cell,
                "battery": battery,
                "battery_before_move": battery_before_move,
                "charging": False,
                "charge_minutes": 0.0,
            }

            if pad_cell is not None and cell == pad_cell:
                item["charging"] = True
                item["battery_before_charge"] = battery
                item["battery_after_charge"] = (
                    trip.charge_to_pct if trip.charge_to_pct is not None else battery
                )
                item["charge_minutes"] = trip.charge_minutes
                battery = item["battery_after_charge"]
                item["battery"] = battery

            steps.append(item)

        return steps

    def deliver(self, trip: Trip) -> Trip:
        """Execute a verified trip and update the drone's final battery/time state."""
        if not trip.verified or trip.driven:
            return trip

        if trip.actual_final_battery is None:
            self._build_actual_consumption(trip)
        left = max(trip.actual_final_battery if trip.actual_final_battery is not None
                   else trip.report.remaining, 0.0)
        gx, gy = self.station.landmarks[trip.destination]
        db.update_drone(trip.drone_id, self.db_path, x=gx, y=gy, battery=left,
                        status="DONE", destination=trip.destination)
        db.update_task_status(trip.task_id, "COMPLETED", self.db_path)
        if trip.plan is not None and trip.pilot_decisions:
            self.adaptive_pilot.learn_from_successful_route(
                trip.plan.cells, trip.pilot_decisions, self.station
            )
            trip.log.append(
                f"Adaptive Pilot learned from successful execution: "
                f"updated {len(trip.pilot_decisions)} Q-learning state/action pair(s)."
            )
        trip.driven = True
        trip.log.append(f"{trip.drone_id} reached the {trip.destination} {(gx, gy)} "
                        f"at {self._format_clock(trip.arrival_minutes or trip.current_minutes)} "
                        f"with {left:.1f}% battery left.")
        if trip.charging_pad:
            trip.log.append(f"Charging stop: {trip.charging_pad.id}, "
                            f"charged to {trip.charge_to_pct:.1f}% in {trip.charge_minutes:.1f} min.")
        db.log_event("TRIP_COMPLETED", trip.log[-1], self.db_path)
        return trip
