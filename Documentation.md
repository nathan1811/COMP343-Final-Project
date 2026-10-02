# Autonomous Drone Routing System — Technical Documentation

## 1. System Overview

The **Autonomous Drone Routing System** is an autonomous routing framework that combines natural-language task interpretation, A\* pathfinding, adaptive movement decisions, formal safety verification using Z3/SMT, perception and state estimation, execution, and SQLite decision logging.

The overall system pipeline is:

**Operator Command → LLM / Fallback Parser → A\* Route Planning → Adaptive Pilot → Time-Aware Scheduling → Z3 / SMT Verification → Execution → Database / Decision Log**

The system supports two input modes:

- Natural-language commands
- Manual drone and route parameters

The central design principle is that the language interface and planning components propose actions, while a separate formal verification layer determines whether those actions are safe to execute.

---

## 2. Natural-Language Task Parsing

For natural-language commands, the system uses **GPT-5-mini** to convert the operator's instruction into a structured task representation.

The parser handles information including:

- Drone ID
- Destination
- Requested deadline
- Other supported task constraints

For multi-drone commands, each explicitly named drone is preserved as a separate task.

For example:

```text
Take Drone-03 to the airport and Drone-04 to the airport
and Drone-01 to the airport and Drone-02 to the airport.
Reach by 11:00 PM.
```

produces separate tasks for Drone-03, Drone-04, Drone-01 and Drone-02.

If the OpenAI API is unavailable, a regex-based fallback parser is available for supported command structures.

The language model proposes the structured task. It does **not** determine whether the resulting movement is safe to execute.

---

## 3. A\* Route Planning

Once the task has been parsed, the A\* pathfinding algorithm calculates a route from each drone's current position to the selected destination.

The station environment is represented as a **10 × 10 grid** containing:

- **Normal cells** — standard movement cost
- **Grey / toll cells** — higher movement cost
- **Obstacles** — impassable cells

A\* uses:

- Manhattan-distance heuristic
- Four-directional movement
- Terrain-dependent movement costs
- Open and closed sets
- `g`, `h` and `f` scores
- Parent-based path reconstruction

The route cost incorporates the additional battery cost associated with grey cells.

The planner can temporarily account for the positions of other drones when generating individual routes, while the final multi-drone schedule is handled separately.

---

## 4. Battery Model

The simulated drone movement uses a random actual battery drain of approximately **1.5%–3.0% per movement step**.

For planning and formal safety verification, the system uses **3.0% as the worst-case movement-drain bound**.

This creates a conservative distinction between:

- Actual simulated battery consumption
- Worst-case planning consumption

Each movement step represents **15 minutes** of simulated time.

The system also maintains a **15% reserve battery requirement** when determining whether a drone can safely complete a delivery or whether charging is required.

The battery model can be represented as:

```text
Battery(i) = Battery(i-1) - Drain × CellCost
```

---

## 5. Adaptive Pilot

The system includes a lightweight **Adaptive Pilot** as an advisory decision layer between route planning and formal verification.

The Adaptive Pilot uses a tabular Q-learning-style structure.

Its state representation includes factors such as:

- Terrain condition
- Local congestion
- Battery band

The available actions are:

```text
PROCEED
CAUTIOUS_WAIT
```

The pilot can therefore decide whether to continue normally or insert a short wait before entering a movement segment.

The Adaptive Pilot is intentionally constrained:

- It does not generate the spatial route.
- It does not replace A\*.
- It cannot remove an obstacle.
- It cannot bypass battery safety constraints.
- It cannot bypass Z3 verification.
- It cannot force execution of an unverified schedule.

This keeps the adaptive layer advisory while preserving the deterministic planning and formal safety architecture.

The conceptual flow is:

```text
A* Route
   ↓
Adaptive Pilot
   ↓
Movement Advice
   ↓
Z3 Verification
   ↓
Execution
```

---

## 6. Automatic Charging

Charging is handled automatically when a drone does not have sufficient battery to safely reach its destination while maintaining the required reserve.

The system first checks whether a direct destination route is feasible.

If additional energy is required, it searches for a suitable charging pad and plans:

```text
Drone → Charging Pad → Destination
```

The required charging target is based on:

- Energy required for the remaining route
- Required reserve battery
- Battery available when the drone reaches the charging pad

The system rejects a charging plan if the required target exceeds 100%.

Charging time is calculated from the required energy and the charging-pad power.

The natural-language operator does not need to manually select a charging percentage.

---

## 7. Multi-Drone Coordination

When several drones are assigned tasks simultaneously, the system creates a time-aware schedule.

The scheduling rules are:

1. Two drones cannot occupy the **same tile at the same time**.
2. Two drones cannot use the **same charging pad at the same time**.
3. Drones may use the same route at different times.
4. A drone may pass through another drone's previous path after that drone has cleared the cell.
5. There is no additional edge-swap restriction.

If a conflict occurs, the scheduler can insert waiting time into a route rather than unnecessarily rejecting the spatial route.

When a conflict requires prioritisation, the starting battery level is used as the priority signal, with the lower-starting-battery drone receiving priority.

The resulting time-expanded schedule is then passed to Z3 for formal verification.

---

## 8. Formal Safety Verification with Z3

After route planning and scheduling, the proposed plan is passed to the **Z3 SMT solver**.

For battery safety, the verifier represents the battery evolution using:

```text
Battery(i) = Battery(i-1) - Drain × CellCost
```

and requires:

```text
Battery(i) ≥ 0
```

for every relevant movement step.

For multi-drone execution, the verifier also checks the time-expanded schedule.

The verifier checks that:

1. A valid route exists.
2. Battery constraints remain satisfied.
3. Two drones are not assigned to the same tile at the same time.
4. Shared charging resources are not simultaneously occupied.
5. The proposed schedule satisfies the required formal constraints.

If the constraints are satisfiable, the plan is marked **verified**.

If the constraints are unsatisfiable, the plan is rejected and the affected drone does not execute the proposed movement.

---

## 9. Execution and Fail-Safe Behaviour

The system follows a **verify-before-execute** architecture.

A proposed trip is first:

1. Parsed
2. Planned using A\*
3. Adapted/scheduled where necessary
4. Verified using Z3
5. Executed only after successful verification

The dashboard provides two primary operations:

### Verify Plan

Calculates the proposed route and verifies it without moving the drone.

### Deliver

Verifies the proposed route and executes it only if verification succeeds.

If verification fails, the drone remains stationary.

This provides a clear separation between decision proposal and state-changing execution.

---

## 10. Perception and State Estimation

The system simulates imperfect drone perception rather than assuming that all sensor measurements are perfectly accurate.

It generates noisy measurements of:

- Battery level
- X-position
- Y-position

A lightweight **1D Kalman filter** is then used to estimate each state variable.

The dashboard displays:

```text
True State → Noisy Measurement → Estimated State
```

This demonstrates how the system can maintain an estimated internal state when simulated sensor observations contain noise.

---

## 11. Database and Decision Logging

The system uses **SQLite** as its persistent memory and logging layer.

The database stores information relating to:

### Drones

- Current battery
- Current position
- Status
- Destination

### Tasks

- Operator commands
- Verification status
- Execution status

### Events

- Timestamped system events
- Decisions
- Verification outcomes
- Execution events

The system records events such as task parsing, verification success or failure, scheduling decisions, and trip completion.

This creates an auditable decision history that can be inspected through the dashboard.

---

## 12. Streamlit Dashboard

The Streamlit dashboard provides the operator interface for the system.

The dashboard includes:

- Natural-language command input
- Manual drone input
- Map seed selection
- Verify Plan control
- Deliver control
- Live station map
- Drone status table
- Charging-pad status
- Multi-drone ETA information
- Verification panel
- AI decision log
- Perception panel
- SQLite database viewer

The default demonstration uses map seed:

```text
8695665
```

and the default natural-language command:

```text
Take Drone-03 to the airport and Drone-04 to the airport and Drone-01 to the airport and Drone-02 to the airport. Reach by 11:00 PM.
```

---

## 13. Overall Architecture

```text
                         Natural Language
                               ↓
                    LLM / Fallback Parser
                               ↓
                      Structured Tasks
                               ↓
                              A*
                               ↓
                    Terrain-Aware Routes
                               ↓
                       Adaptive Pilot
                               ↓
                  Time-Aware Scheduling
                               ↓
                          Z3 / SMT
                               ↓
                         Verified?
                       ↙         ↘
                    Reject      Execute
                                  ↓
                         SQLite / Dashboard
```

The architecture deliberately separates task interpretation, route generation, adaptive movement advice, scheduling, formal verification, execution, and memory.

The language model and route planner can propose actions, but **formal verification acts as the safety gate before execution**.

---

## 14. Algorithmic Foundations

The project uses established algorithmic techniques as foundations for its system components.

The main technical foundations include:

- **A\*** for grid-based route planning and terrain-aware cost calculation.
- **Structured language parsing** for converting operator instructions into machine-readable tasks.
- **Q-learning-style tabular decision making** for the Adaptive Pilot.
- **SMT/Z3 constraint solving** for formal route, battery and multi-drone safety verification.
- **Kalman filtering** for noisy state estimation.
- **SQLite** for persistent memory and event logging.

These components are integrated as separate layers so that adaptive or probabilistic decision-making does not replace the formal verification layer.

---

## 15. Key Design Principle

The system separates **proposal** from **execution**:

```text
LLM / Parser
      ↓
Proposes structured task
      ↓
A*
      ↓
Proposes route
      ↓
Adaptive Pilot
      ↓
Provides movement advice
      ↓
Time-Aware Scheduler
      ↓
Constructs executable schedule
      ↓
Z3 / SMT
      ↓
Formally verifies safety
      ↓
Execution
```

The LLM is therefore not trusted to directly control the drone.

A proposed action must pass the formal verification layer before it can change the simulated drone state.

This separation allows the project to combine natural-language interaction, classical planning, adaptive decision-making, perception, and formal verification within a single autonomous routing framework.
