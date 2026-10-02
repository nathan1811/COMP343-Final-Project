"""
dashboard.py  --  run with:  streamlit run dashboard.py

Sidebar: choose the simulated time of day and operator command. Natural-language
commands may include a deadline ('by 10:30 AM') and charging target ('charge to 80%').
Each movement step takes 15 minutes. If the drone cannot reach its destination,
the planner selects a viable charging pad and computes charging time from pad power.
"""
import random
import time
from datetime import time as dt_time

import matplotlib.pyplot as plt
import streamlit as st

import llm_parser
from orchestrator import StationManager, STEP_MINUTES

st.set_page_config(page_title="Autonomous Drone Delivery", layout="wide")

APP_VERSION = 11

if st.session_state.get("app_version") != APP_VERSION:
    st.session_state.clear()
    st.session_state.app_version = APP_VERSION
    st.session_state.manager = StationManager(reset=True)
    st.session_state.manager.load_scenario(2026)
    st.session_state.seed = 2026
    st.session_state.trip = None
    st.session_state.multi_trip = None

sm = st.session_state.manager
st.title("Autonomous Drone Delivery")
st.caption("LLM proposes -> A* calculates -> charging is planned if needed -> SMT verifies -> System executes.")



def load_map(seed: int):
    st.session_state.seed = seed
    sm.load_scenario(seed)
    st.session_state.trip = None
    st.session_state.multi_trip = None
    st.rerun()


def time_to_minutes(t: dt_time) -> float:
    return t.hour * 60 + t.minute


def draw_station(station, drones, charging_pads, planned=None, driven=False,
                 used_pad_id=None, used_pad_ids=None, show_planned=False, current_cell=None,
                 status_text=None, planned_routes=None):
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    for y in range(station.HEIGHT):
        for x in range(station.WIDTH):
            cell = station.grid[y][x]
            if cell == station.OBSTACLE:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#3b3b3b"))
            elif cell == station.GREY:
                ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, color="#c9c9c9"))

    if planned_routes:
        for route in planned_routes:
            if len(route) > 1:
                xs, ys = [p[0] for p in route], [p[1] for p in route]
                ax.plot(xs, ys, linewidth=2.0 if driven else 1.5,
                        linestyle="-" if driven else ":", alpha=0.8, zorder=5)
    elif planned and len(planned) > 1:
        xs, ys = [p[0] for p in planned], [p[1] for p in planned]
        if driven:
            ax.plot(xs, ys, color="#1a73e8", linewidth=2.8, marker="o", markersize=3.5,
                    zorder=6, label="route delivered")
        elif show_planned:
            ax.plot(xs, ys, color="#9e9e9e", linewidth=1.8, linestyle=":", zorder=1,
                    label="attempted route")
        if driven or show_planned:
            ax.legend(loc="upper left", fontsize=7, framealpha=0.8)

    for c in charging_pads:
        is_used = (used_pad_id == c.id) or (used_pad_ids is not None and c.id in used_pad_ids)
        pad_color = "#e53935" if is_used else "#34a853"
        ax.scatter([c.x], [c.y], marker="s", s=300, color=pad_color, zorder=2)
        label = f"{c.id} {c.power_kw:.0f}kW"
        if is_used:
            label += " USED"
        ax.text(c.x, c.y - 0.4, label, ha="center", fontsize=7)

    ax.scatter([station.supermarket[0]], [station.supermarket[1]], marker="D", s=330,
               color="#ff9800", edgecolors="#8a4b00", zorder=3)
    ax.text(station.supermarket[0], station.supermarket[1] - 0.5, "SUPERMARKET",
            ha="center", fontsize=8)

    ax.scatter([station.airport[0]], [station.airport[1]], marker="*", s=520,
               color="#f1c232", edgecolors="#7f6000", zorder=3)
    ax.text(station.airport[0], station.airport[1] - 0.5, "AIRPORT", ha="center", fontsize=8)

    for v in drones:
        ax.scatter([v["x"]], [v["y"]], marker="o", s=230, color="#4285f4", zorder=7)
        ax.text(v["x"], v["y"] + 0.35, f"{v['id']} ({v['battery']:.0f}%)",
                ha="center", fontsize=7.5, zorder=8)

    if current_cell is not None:
        ax.scatter([current_cell[0]], [current_cell[1]], marker="o", s=310,
                   facecolors="none", edgecolors="#111111", linewidths=2.0, zorder=9)

    if status_text:
        ax.text(0.02, 0.98, status_text, transform=ax.transAxes, va="top",
                fontsize=9, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.85))

    ax.set_xlim(-1, station.WIDTH)
    ax.set_ylim(-1, station.HEIGHT)
    ax.invert_yaxis()
    ax.set_xticks(range(station.WIDTH))
    ax.set_yticks(range(station.HEIGHT))
    ax.grid(True, linewidth=0.3, color="#dddddd")
    ax.set_aspect("equal")
    return fig


def start_delivery_animation(trip):
    """Store the verified trip so the existing Station Map can animate it in-place."""
    if not trip.verified:
        return False

    steps = sm.get_delivery_animation(trip)
    if not steps:
        return False

    st.session_state.animation = {
        "steps": steps,
        "index": 0,
        "visited": [],
        "used_pad_id": None,
        "trip": trip,
    }
    return True


def _animated_drones(trip, cell, battery):
    """Overlay the moving drone on the current map without changing the database."""
    return [
        {**d, "x": cell[0], "y": cell[1], "battery": battery}
        if d["id"] == trip.drone_id else d
        for d in sm.get_drones()
    ]


def _multi_state_at_tick(multi, tick):
    """Return positions/batteries and the charger actively occupied at a tick."""
    drones = {d["id"]: dict(d) for d in sm.get_drones()}
    charging_now = []
    for trip in multi.trips:
        cells = trip.execution_cells or []
        if not cells:
            continue
        idx = min(tick, len(cells) - 1)
        cell = tuple(cells[idx])
        battery = trip.start_battery
        pad_cell = ((trip.charging_pad.x, trip.charging_pad.y)
                    if trip.charging_pad else None)
        pad_seen = False
        drain_index = 0
        step_drains = trip.step_drains or []
        for i in range(1, idx + 1):
            prev = tuple(cells[i - 1])
            cur = tuple(cells[i])
            if cur != prev:
                actual_drain = (
                    step_drains[drain_index]
                    if drain_index < len(step_drains)
                    else 3.0
                )
                drain_index += 1
                battery -= sm.station.cost(*cur) * actual_drain
            if pad_cell is not None and cur == pad_cell and not pad_seen:
                pad_seen = True
                if trip.charge_minutes > 0:
                    battery = trip.charge_to_pct
            if pad_cell is not None and cur == pad_cell and pad_seen and i < len(cells) - 1:
                # Repeated pad cells represent the charging interval.
                if trip.charge_minutes > 0:
                    charging_now.append(trip.charging_pad.id)
        if trip.charging_pad and pad_cell == cell and trip.charge_minutes > 0:
            charging_now.append(trip.charging_pad.id)
        drones[trip.drone_id].update({"x": cell[0], "y": cell[1], "battery": max(0.0, battery)})
    return list(drones.values()), sorted(set(charging_now))


def start_multi_delivery_animation(multi):
    if not multi.verified:
        return False
    if not multi.trips:
        return False
    max_steps = max(len(t.execution_cells or []) for t in multi.trips)
    if max_steps == 0:
        return False
    st.session_state.multi_animation = {
        "tick": 0,
        "multi": multi,
        "max_tick": max_steps - 1,
    }
    return True


with st.sidebar:
    with st.expander("LLM connection check"):
        if st.button("Test LLM connection", use_container_width=True):
            r = llm_parser.test_llm_connection()
            st.success(f"{r['model']} replied: {r['reply']!r}") if r["ok"] else st.error(r["reason"])

    st.header("Simulation Time")
    current_time = st.time_input("Current time", value=dt_time(9, 0), step=900,
                                  help="Every movement step takes 15 minutes.")
    current_minutes = time_to_minutes(current_time)
    st.caption(f"Movement step = {STEP_MINUTES:g} minutes")

    st.header("Operator Command")
    mode = st.radio("Input mode", ["Natural language", "Manual entry"], horizontal=True)

    if mode == "Natural language":
        command = st.text_area(
            "Instruction", height=150,
            value=("Take Drone-01 to the supermarket and Drone-02 to the airport. "
                   "Reach by 11:00 AM."),
        )
        st.info(
            "Use one natural-language command for one or many drones. "
            "Each movement step randomly consumes 1.5%-3.0% battery. "
            "The planner uses 3.0% as the worst-case safety bound and automatically "
            "coordinates routes, collisions, and shared charging pads."
        )
        st.caption("Charging is automatic: each drone is charged only as much as needed to finish with at least 15% battery.")
    else:
        drone_id = st.selectbox("Drone to deliver", [v["id"] for v in sm.get_drones()])
        st.info(
            "Battery use is automatic: each movement step randomly consumes "
            "1.5%-3.0%. The planner uses 3.0% as the worst-case safety bound."
        )
        drain = 3.0  # retained for API compatibility; orchestrator normalizes it
        destination = st.selectbox("Destination", ["AIRPORT", "SUPERMARKET"])
        deadline_enabled = st.checkbox("Use delivery deadline")
        deadline_time = st.time_input("Reach destination by", value=dt_time(11, 0), step=900,
                                      disabled=not deadline_enabled)
        st.info("If charging is required, the system automatically calculates the charge needed to arrive with at least 15% battery reserve.")
        st.caption("Path-finding: **A-star**.")

    st.divider()
    col_seed, col_roll = st.columns([2, 1])
    seed = col_seed.number_input("Map seed", 0, 10_000_000, int(st.session_state.seed), step=1)
    col_roll.write("")
    if col_roll.button("New map", use_container_width=True):
        load_map(random.randint(0, 10_000_000))
    if int(seed) != int(sm.seed if sm.seed is not None else -1):
        if st.button(f"Load map {int(seed)}", use_container_width=True):
            load_map(int(seed))

    st.divider()

    def make_trip():
        if mode == "Natural language":
            return sm.plan_from_command(command, current_minutes=current_minutes)
        return sm.plan_trip(
            drone_id, float(drain), destination,
            current_minutes=current_minutes,
            deadline_text=deadline_time.strftime("%I:%M %p") if deadline_enabled else None,
            charge_to_pct=None,
        )

    c1, c2 = st.columns(2)
    if c1.button("Verify plan", use_container_width=True):
        parsed_plan = make_trip()
        if hasattr(parsed_plan, "trips"):
            st.session_state.multi_trip = parsed_plan
            st.session_state.trip = None
        else:
            st.session_state.trip = parsed_plan
            st.session_state.multi_trip = None
        st.session_state.animation = None
        st.session_state.multi_animation = None
        st.rerun()
    if c2.button("Deliver ▶", type="primary", use_container_width=True):
        parsed_plan = make_trip()
        st.session_state.animation = None
        st.session_state.multi_animation = None
        if hasattr(parsed_plan, "trips"):
            st.session_state.multi_trip = parsed_plan
            st.session_state.trip = None
            if parsed_plan.verified:
                start_multi_delivery_animation(parsed_plan)
            else:
                st.error("❌ Multi-drone delivery cannot start because the natural-language plan was not fully verified.")
        else:
            st.session_state.trip = parsed_plan
            st.session_state.multi_trip = None
            if parsed_plan.verified:
                start_delivery_animation(parsed_plan)
            else:
                st.error("❌ Delivery cannot start because the natural-language plan was not verified. Check the Verification Panel below.")

    st.caption(f"Current map: {sm.scenario.describe()}")
    if st.button("Reset drones (same map)", use_container_width=True):
        load_map(int(sm.seed) if sm.seed is not None else 2026)



trip = st.session_state.trip
multi_trip = st.session_state.get("multi_trip")
col_map, col_status = st.columns([1.1, 1])

with col_map:
    st.subheader("Station Map")
    map_slot = st.empty()
    status_slot = st.empty()

    multi_animation = st.session_state.get("multi_animation")
    animation = st.session_state.get("animation")
    if multi_animation:
        multi = multi_animation["multi"]
        max_tick = multi_animation["max_tick"]

        # Animate the whole fleet in one Streamlit run. This keeps the page
        # itself stable: only the Station Map and its status placeholder update.
        for tick in range(multi_animation["tick"], max_tick + 1):
            drones_now, charging_ids = _multi_state_at_tick(multi, tick)

            # Only draw the portion of each route that has already been travelled.
            # The future route stays invisible until the drone actually reaches it.
            visible_routes = []
            for t in multi.trips:
                cells = t.execution_cells or []
                if cells:
                    visible_routes.append(cells[:min(tick + 1, len(cells))])

            fig = draw_station(
                sm.station, drones_now, sm.scenario.charging_pads,
                driven=True, planned_routes=visible_routes,
                used_pad_ids=charging_ids,
                status_text=f"MULTI-DRONE DELIVERY — T+{tick * STEP_MINUTES:g} MIN"
            )
            map_slot.pyplot(fig, use_container_width=True)
            plt.close(fig)

            active = [t.drone_id for t in multi.trips
                      if t.execution_cells and tick < len(t.execution_cells) - 1]
            status_slot.info(
                f"🚁 Coordinating {len(multi.trips)} drones — "
                f"{', '.join(active) if active else 'all deliveries complete'}"
            )
            if charging_ids:
                status_slot.warning(f"🔴 Charging pad in use: {', '.join(charging_ids)}")

            time.sleep(0.4)

        multi_done = sm.deliver_multi(multi)
        st.session_state.multi_trip = multi_done
        st.session_state.multi_animation = None
        status_slot.success("✅ Multi-drone delivery complete — all verified drones reached their destinations.")
        st.rerun()

    elif animation:
        trip_anim = animation["trip"]
        steps = animation["steps"]
        visited = animation["visited"]
        used_pad_id = animation["used_pad_id"]

        # Animate directly inside the ORIGINAL Station Map placeholder.
        # No second map, no resizing, and no page-level animation panel.
        for idx in range(animation["index"], len(steps)):
            step = steps[idx]
            cell = tuple(step["cell"])

            if not visited or tuple(visited[-1]) != cell:
                visited.append(cell)

            if step.get("charging"):
                used_pad_id = trip_anim.charging_pad.id if trip_anim.charging_pad else None
                animation["used_pad_id"] = used_pad_id
                before = step.get("battery_before_charge", step["battery"])

                drones_now = _animated_drones(trip_anim, cell, before)
                fig = draw_station(
                    sm.station, drones_now, sm.scenario.charging_pads,
                    planned=visited, driven=True, used_pad_id=used_pad_id,
                    current_cell=cell, status_text=f"STOPPED AT {used_pad_id} — CHARGING"
                )
                map_slot.pyplot(fig, use_container_width=True)
                plt.close(fig)
                status_slot.warning(
                    f"🔴 **{used_pad_id} reached** — {trip_anim.drone_id} stopped here for charging. "
                    f"Battery: {before:.1f}%"
                )
                time.sleep(1.0)

                after = step.get("battery_after_charge", step["battery"])
                drones_now = _animated_drones(trip_anim, cell, after)
                fig = draw_station(
                    sm.station, drones_now, sm.scenario.charging_pads,
                    planned=visited, driven=True, used_pad_id=used_pad_id,
                    current_cell=cell, status_text=f"CHARGED AT {used_pad_id} — CONTINUING"
                )
                map_slot.pyplot(fig, use_container_width=True)
                plt.close(fig)
                status_slot.success(
                    f"⚡ **{used_pad_id} charged** — {before:.1f}% → {after:.1f}%. "
                    f"Continuing to {trip_anim.destination}."
                )
                time.sleep(0.35)
            else:
                battery = step["battery"]
                drones_now = _animated_drones(trip_anim, cell, battery)
                fig = draw_station(
                    sm.station, drones_now, sm.scenario.charging_pads,
                    planned=visited, driven=True, used_pad_id=used_pad_id,
                    current_cell=cell, status_text="DELIVERING"
                )
                map_slot.pyplot(fig, use_container_width=True)
                plt.close(fig)
                status_slot.info(
                    f"🚁 **{trip_anim.drone_id} moving** — step {idx}/{len(steps)-1} "
                    f"• position {cell} • battery {battery:.1f}%"
                )
                time.sleep(0.4)

            animation["index"] = idx + 1

        # Animation has reached the final destination. Commit the actual state once.
        trip_done = sm.deliver(trip_anim)
        st.session_state.trip = trip_done
        st.session_state.animation = None
        status_slot.success(
            f"✅ **Delivery complete** — {trip_done.drone_id} reached {trip_done.destination}."
        )
        st.rerun()

    else:
        if multi_trip:
            fig = draw_station(
                sm.station, sm.get_drones(), sm.scenario.charging_pads,
                planned_routes=[t.route_cells for t in multi_trip.trips if t.route_cells],
                driven=all(t.driven for t in multi_trip.trips if t.verified),
                used_pad_ids=None,
            )
            map_slot.pyplot(fig, use_container_width=True)
            plt.close(fig)
        else:
            battery_failed = False
            if trip and trip.report and not trip.report.plan_ok:
                battery_failed = any(
                    (not check.passed) and (
                        "battery" in check.name.lower()
                        or "charging" in check.name.lower()
                        or "route exists" in check.name.lower()
                    )
                    for check in trip.report.checks
                )
            show_attempted_route = bool(trip and not trip.driven and battery_failed and trip.plan)
            fig = draw_station(
                sm.station, sm.get_drones(), sm.scenario.charging_pads,
                (trip.route_cells if trip and trip.route_cells else
                 (trip.plan.cells if trip and trip.plan else None)),
                driven=bool(trip and trip.driven),
                # A charging pad is highlighted red only during the live delivery
                # animation. A verified plan should not show a red pad when no
                # delivery route is currently being drawn.
                used_pad_id=None,
                show_planned=show_attempted_route,
            )
            map_slot.pyplot(fig, use_container_width=True)
            plt.close(fig)
            st.caption("Star = airport, orange diamond = supermarket, green = charging pad. "
                       "Black = no-fly obstacles, light grey = high-drain cells. "
                       "Dotted = attempted route only when the plan cannot be completed; "
                       "solid blue = delivered route. A pad turns red only while the drone is actively charging.")

with col_status:
    st.subheader("Drone Status")
    st.dataframe([{"ID": v["id"], "Battery %": round(v["battery"], 1),
                   "Pos": f"({v['x']},{v['y']})", "Status": v["status"],
                   "Destination": v["destination"] or "-"}
                  for v in sm.get_drones()], use_container_width=True, hide_index=True)

    st.subheader("Charging Pads")
    st.dataframe([{"ID": c.id, "Type": c.type, "Power (kW)": c.power_kw,
                   "Pos": f"({c.x},{c.y})"} for c in sm.scenario.charging_pads],
                 use_container_width=True, hide_index=True)

    if multi_trip:
        st.subheader("Multi-Drone ETA")
        for t in multi_trip.trips:
            if t.report and t.arrival_minutes is not None:
                if t.deadline_minutes is not None and t.arrival_minutes > t.deadline_minutes:
                    st.error(f"{t.drone_id}: cannot reach {t.destination} in time — ETA {sm._format_clock(t.arrival_minutes)} vs {t.deadline_text}.")
                elif t.verified:
                    st.success(f"{t.drone_id}: ETA **{sm._format_clock(t.arrival_minutes)}** → {t.destination}")
                else:
                    failed = next((c for c in (t.report.checks if t.report else [])
                                   if not c.passed and c.name not in {
                                       "Charging pad selected",
                                       "Reach charging pad",
                                       "Charging",
                                       "Route to destination",
                                   }), None)
                    reason = failed.detail if failed else "plan could not be verified"
                    st.error(f"{t.drone_id}: {reason}")

    if trip and trip.report:
        m1, m2, m3, m4 = st.columns(4)
        m1.metric("Battery at start", f"{trip.start_battery:.1f}%")
        m2.metric("Trip energy", f"{trip.report.needed:.1f}%")
        m3.metric("Worst-case arrival", f"{trip.report.remaining:.1f}%")
        if trip.actual_final_battery is not None:
            st.caption(f"Actual simulated final battery: **{trip.actual_final_battery:.1f}%**")
        m4.metric("Estimated arrival",
                  sm._format_clock(trip.arrival_minutes) if trip.arrival_minutes is not None else "-")

        # Deadline is a delivery constraint, not a charging-status display.
        if trip.deadline_minutes is not None and trip.arrival_minutes is not None:
            if trip.arrival_minutes > trip.deadline_minutes:
                st.error(
                    f"❌ Cannot reach the final destination in time. "
                    f"Estimated arrival: **{sm._format_clock(trip.arrival_minutes)}** "
                    f"vs deadline: **{trip.deadline_text}**."
                )
            else:
                st.success(
                    f"✅ Estimated arrival: **{sm._format_clock(trip.arrival_minutes)}** "
                    f"— before the **{trip.deadline_text}** deadline."
                )

st.divider()

def display_decision_log(log):
    """Hide internal charging-selection details from the operator-facing log."""
    hidden_phrases = (
        "Charging decision:",
        "A* route via",
        "Charging pad selected",
        "Reach charging pad",
        "Charging --",
        "Route to destination --",
    )
    return "\n".join(
        line for line in log
        if not any(p.lower() in line.lower() for p in hidden_phrases)
    )


col_v, col_log = st.columns([1, 1.3])
with col_v:
    st.subheader("Verification Panel")
    if multi_trip:
        all_ok = multi_trip.verified
        for t in multi_trip.trips:
            if t.report:
                st.markdown(f"**{t.drone_id} → {t.destination}**")
                hidden_checks = {
                    "Charging pad selected",
                    "Reach charging pad",
                    "Charging",
                    "Route to destination",
                }
                for c in t.report.checks:
                    if c.name in hidden_checks:
                        continue
                    st.markdown(f"{'✅' if c.passed else '❌'} **{c.name}** — {c.detail}")
                if t.deadline_minutes is not None and t.arrival_minutes is not None and t.arrival_minutes > t.deadline_minutes:
                    st.error(
                        f"Cannot reach final destination in time — ETA {sm._format_clock(t.arrival_minutes)} "
                        f"vs deadline {t.deadline_text}."
                    )
                elif t.verified:
                    st.success(f"ETA **{sm._format_clock(t.arrival_minutes)}** — route verified.")
                else:
                    failed = next((c for c in (t.report.checks if t.report else [])
                                   if not c.passed and c.name not in hidden_checks), None)
                    if failed:
                        st.error(f"{failed.name}: {failed.detail}")
                    else:
                        st.error("This drone could not be scheduled or formally verified.")
                st.divider()
        if all_ok:
            st.success("🟢 **ALL DRONE PLANS VERIFIED (SAFE)** — routes are coordinated and shared chargers are scheduled.")
        else:
            st.error("🔴 **MULTI-DRONE PLAN REJECTED** — at least one drone could not be safely scheduled.")
    elif trip is None:
        st.info("Verify a plan to see the SMT result.")
    elif trip.report:
        hidden_checks = {
            "Charging pad selected",
            "Reach charging pad",
            "Charging",
            "Route to destination",
        }
        for c in trip.report.checks:
            if c.name in hidden_checks:
                continue
            st.markdown(f"{'✅' if c.passed else '❌'} **{c.name}** — {c.detail}")

        if (trip.deadline_minutes is not None and
                trip.arrival_minutes is not None and
                trip.arrival_minutes > trip.deadline_minutes):
            st.error(
                f"❌ **Cannot reach final destination in time** — "
                f"estimated arrival {sm._format_clock(trip.arrival_minutes)}, "
                f"deadline {trip.deadline_text}."
            )
        elif trip.report.plan_ok:
            st.success(
                f"🟢 **PLAN VERIFIED (SAFE)** — estimated arrival "
                f"{sm._format_clock(trip.arrival_minutes)}."
            )
        else:
            st.markdown("### 🔴 PLAN REJECTED")
with col_log:
    st.subheader("AI Decision Log")
    if multi_trip:
        combined_log = []
        for t in multi_trip.trips:
            combined_log.append(f"[{t.drone_id}] "+"\n".join(display_decision_log(t.log)))
        st.code("\n\n".join(combined_log), language=None)
    elif trip:
        st.code(display_decision_log(trip.log), language=None)
        if trip.react_trace:
            with st.expander("ReAct parser trace"):
                st.code("\n".join(f"{m['role']}: {m['content']}" for m in trip.react_trace), language=None)
    else:
        st.info("No commands yet.")

st.divider()
st.subheader("Perception: True → Noisy → Estimated")
st.dataframe([{
    "Drone": vid,
    "True Battery": round(s["true"]["battery"], 2),
    "Noisy Battery": s["noisy"]["battery"],
    "Estimated Battery": s["estimated"]["battery"],
    "True Pos": f"({s['true']['x']:.1f}, {s['true']['y']:.1f})",
    "Noisy Pos": f"({s['noisy']['x']:.1f}, {s['noisy']['y']:.1f})",
    "Estimated Pos": f"({s['estimated']['x']:.1f}, {s['estimated']['y']:.1f})",
} for vid, s in sm.perception_snapshot().items()], use_container_width=True, hide_index=True)

st.divider()
st.subheader("Database")
t1, t2, t3 = st.tabs(["Tasks", "Events", "Drones"])
t1.dataframe(sm.get_tasks(), use_container_width=True, hide_index=True)
t2.dataframe(sm.get_events(50), use_container_width=True, hide_index=True)
t3.dataframe(sm.get_drones(), use_container_width=True, hide_index=True)
