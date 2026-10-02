from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import re

import z3
from pydantic import BaseModel, Field


class RouteCell(BaseModel):
    step: int = Field(ge=0)
    x: int
    y: int
    energy_cost: float = Field(ge=0)


class DroneSpec(BaseModel):
    id: str
    battery_pct: float = Field(ge=0, le=100)
    min_battery: float = Field(default=0, ge=0, le=100)


class VerificationPayload(BaseModel):
    drone: DroneSpec
    route: List[RouteCell]


@dataclass
class CheckResult:
    name: str
    passed: bool
    detail: str


@dataclass
class VerificationReport:
    plan_ok: bool
    checks: List[CheckResult] = field(default_factory=list)
    needed: float = 0.0
    remaining: float = 0.0
    charge_needed: float = 0.0
    charge_minutes: float = 0.0
    arrival_minutes: Optional[float] = None

    def add(self, name: str, passed: bool, detail: str):
        self.checks.append(CheckResult(name, passed, detail))
        if not passed:
            self.plan_ok = False

    def as_lines(self) -> List[str]:
        lines = [f"[{'PASS' if c.passed else 'FAIL'}] {c.name} -- {c.detail}"
                 for c in self.checks]
        lines.append("RESULT: PLAN VERIFIED (SAFE)" if self.plan_ok else "RESULT: PLAN REJECTED")
        return lines


def _q(value: float):
    return z3.Q(int(round(value * 1000)), 1000)


def verify_route_smt(payload: Dict[str, Any]) -> Dict[str, Any]:
    try:
        data = VerificationPayload.model_validate(payload)
    except Exception as exc:
        return {"status": "SYNTACTIC_ERROR", "message": str(exc)}

    solver = z3.Solver()
    previous = _q(data.drone.battery_pct)
    battery_vars = []

    for cell in data.route:
        battery = z3.Real(f"battery_{cell.step}")
        solver.add(battery == previous - _q(cell.energy_cost))
        solver.add(battery >= _q(data.drone.min_battery))
        battery_vars.append(battery)
        previous = battery

    if solver.check() == z3.sat:
        model = solver.model()
        if battery_vars:
            text = model[battery_vars[-1]].as_decimal(12).rstrip("?")
            final_value = float(text[:-2]) if text.endswith("/1") else float(text)
        else:
            final_value = data.drone.battery_pct
        return {
            "status": "SATISFIABLE",
            "remaining": final_value,
            "feedback": "Route battery constraints verified mathematically.",
        }

    return {
        "status": "UNSATISFIABLE",
        "feedback": "The Drone battery would fall below its minimum allowed level on this route.",
    }


def verify_trip(drone, plan_to_destination, station, drain_per_cell: float,
                destination: str = "AIRPORT", charging_plan: Optional[Dict[str, Any]] = None,
                deadline_ok: Optional[bool] = None, arrival_minutes: Optional[float] = None,
                deadline_text: Optional[str] = None) -> VerificationReport:
    report = VerificationReport(plan_ok=True, arrival_minutes=arrival_minutes)

    if charging_plan:
        outbound = charging_plan["to_pad_plan"]
        final_leg = charging_plan["to_destination_plan"]
        pad = charging_plan["pad"]
        charge_to = charging_plan["charge_to_pct"]
        battery_at_pad = charging_plan["battery_at_pad"]
        report.charge_needed = charging_plan["charge_needed"]
        report.charge_minutes = charging_plan["charge_minutes"]
        combined_steps = outbound.steps + final_leg.steps
        report.add("Charging pad selected", True,
                   f"{pad.id} ({pad.power_kw:g} kW) at ({pad.x},{pad.y})")
        report.add("Reach charging pad", True,
                   f"{outbound.steps} steps, battery reaches {battery_at_pad:.1f}%")
        report.add("Charging", True,
                   f"charge from {battery_at_pad:.1f}% to {charge_to:.1f}% "
                   f"({report.charge_minutes:.1f} minutes)")
        report.add("Route to destination", True,
                   f"{final_leg.steps} steps after charging")
        report.needed = (outbound.cost + final_leg.cost) * drain_per_cell
        report.remaining = charge_to - final_leg.cost * drain_per_cell

        route = []
        step = 1
        for cell in outbound.cells[1:]:
            route.append(RouteCell(step=step, x=cell[0], y=cell[1],
                                   energy_cost=drain_per_cell * station.cost(*cell)))
            step += 1
        for cell in final_leg.cells[1:]:
            route.append(RouteCell(step=step, x=cell[0], y=cell[1],
                                   energy_cost=drain_per_cell * station.cost(*cell)))
            step += 1

        # Verify the outbound leg from the starting battery.
        outbound_payload = {
            "drone": {"id": drone.id, "battery_pct": drone.battery, "min_battery": 0},
            "route": [c.model_dump() for c in route[:outbound.steps]],
        }
        out_result = verify_route_smt(outbound_payload)
        out_ok = out_result["status"] == "SATISFIABLE"
        report.add("Battery reaches charging pad", out_ok,
                   f"has {drone.battery:.1f}% -> needs {outbound.cost * drain_per_cell:.1f}%")

        # Verify the final leg after charging.
        final_start = charge_to
        final_payload = {
            "drone": {"id": drone.id, "battery_pct": final_start, "min_battery": 0},
            "route": [c.model_dump() for c in route[outbound.steps:]],
        }
        final_result = verify_route_smt(final_payload)
        final_ok = final_result["status"] == "SATISFIABLE"
        report.add(f"Battery reaches the {destination.lower()}", final_ok,
                   f"starts final leg at {charge_to:.1f}%, needs "
                   f"{final_leg.cost * drain_per_cell:.1f}%, "
                   f"arrives with {report.remaining:.1f}%")

        if deadline_ok is not None:
            detail = (f"arrival at {arrival_minutes:.0f} minutes from midnight"
                      if arrival_minutes is not None else "arrival time unavailable")
            if deadline_text:
                detail += f", deadline {deadline_text}"
            report.add("Delivery deadline", deadline_ok, detail)
        return report

    if plan_to_destination is None:
        report.add("Route exists", False,
                   f"no route to the {destination.lower()} (blocked by obstacles or parked drones)")
        return report

    report.add("Route exists", True,
                f"{plan_to_destination.steps} steps, {plan_to_destination.grey_cells} toll cell(s)")
    total_cost = plan_to_destination.cost * drain_per_cell
    report.needed = total_cost
    report.remaining = drone.battery - total_cost

    route = [
        RouteCell(step=i, x=cell[0], y=cell[1],
                  energy_cost=drain_per_cell * station.cost(*cell))
        for i, cell in enumerate(plan_to_destination.cells[1:], start=1)
    ]
    payload = {
        "drone": {"id": drone.id, "battery_pct": drone.battery, "min_battery": 0},
        "route": [cell.model_dump() for cell in route],
    }
    result = verify_route_smt(payload)
    ok = result["status"] == "SATISFIABLE"

    if ok:
        detail = (f"has {drone.battery:.1f}%, trip needs {report.needed:.1f}% "
                  f"({drain_per_cell:g}% per cell) -> arrives with {report.remaining:.1f}%")
    else:
        level = drone.battery
        failed_cell = None
        failed_step = None
        for i, cell in enumerate(plan_to_destination.cells[1:], start=1):
            level -= drain_per_cell * station.cost(*cell)
            if level < 0:
                failed_cell, failed_step = cell, i
                break
        detail = (f"has {drone.battery:.1f}%, trip needs {report.needed:.1f}% "
                  f"({-report.remaining:.1f}% short) -- would run flat at "
                  f"cell {failed_cell} (step {failed_step} of {plan_to_destination.steps}).")

    report.add(f"Battery reaches the {destination.lower()}", ok, detail)
    if deadline_ok is not None:
        report.add("Delivery deadline", deadline_ok,
                   f"arrival at {arrival_minutes:.0f} minutes from midnight, deadline {deadline_text}")
    return report


def verify_multi_drone_schedule_smt(
    schedules: Dict[str, List[Any]],
    charger_cells: Optional[Dict[str, Tuple[int, int]]] = None,
) -> Dict[str, Any]:
    """Formally verify the time-expanded multi-drone schedule with Z3.

    Routes may overlap freely. The only spatial conflict is two drones being
    on the same tile at the same simulation tick. A charger is the same kind
    of shared tile, so simultaneous use of one charger is also rejected.
    Once a drone reaches its destination, its schedule ends; it does not
    reserve that tile forever.
    """
    schedules = {
        str(drone_id): [tuple(cell) for cell in route]
        for drone_id, route in schedules.items()
        if route
    }
    charger_cells = charger_cells or {}

    if not schedules:
        return {
            "status": "SATISFIABLE",
            "safe": True,
            "conflicts": [],
            "feedback": "No multi-drone schedule to verify.",
        }

    solver = z3.Solver()
    ids = list(schedules)
    positions = {}

    for drone_id, route in schedules.items():
        for tick, cell in enumerate(route):
            x = z3.Int(f"md_x_{_safe_name(drone_id)}_{tick}")
            y = z3.Int(f"md_y_{_safe_name(drone_id)}_{tick}")
            solver.add(x == int(cell[0]))
            solver.add(y == int(cell[1]))
            positions[(drone_id, tick)] = (x, y)

    conflicts = []
    for i, a in enumerate(ids):
        for b in ids[i + 1:]:
            overlap = min(len(schedules[a]), len(schedules[b]))
            for tick in range(overlap):
                ax, ay = positions[(a, tick)]
                bx, by = positions[(b, tick)]
                same_tile = z3.And(ax == bx, ay == by)
                solver.add(z3.Not(same_tile))

                if schedules[a][tick] == schedules[b][tick]:
                    cell = schedules[a][tick]
                    conflict = {
                        "type": "tile",
                        "tick": tick,
                        "drones": (a, b),
                        "cell": cell,
                    }
                    for pad_id, pad_cell in charger_cells.items():
                        if cell == tuple(pad_cell):
                            conflict["type"] = "charger"
                            conflict["charger"] = pad_id
                            break
                    conflicts.append(conflict)

    if solver.check() == z3.sat:
        return {
            "status": "SATISFIABLE",
            "safe": True,
            "conflicts": [],
            "feedback": "Z3 verified that no two active drones occupy the same tile at the same time.",
        }

    unique = []
    seen = set()
    for conflict in conflicts:
        key = repr(sorted(conflict.items()))
        if key not in seen:
            seen.add(key)
            unique.append(conflict)

    return {
        "status": "UNSATISFIABLE",
        "safe": False,
        "conflicts": unique,
        "feedback": "Z3 found two active drones occupying the same tile at the same time.",
    }

def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", str(value))
