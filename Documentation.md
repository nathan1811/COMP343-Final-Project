Autonomous Drone Routing System — Technical Documentation

1. System Overview

The system is an autonomous Drone routing framework that combines natural-language task interpretation, A\* pathfinding, formal safety verification using Z3, state estimation, and database logging.

Overall pipeline:

Operator Command → LLM Parser → A\* Route Planning → Z3 Verification → Execution → Database/Decision Log

The system supports two input modes: natural-language commands and manual drone/route parameters.

2. Natural-Language Task Parsing

For natural-language commands, the system uses GPT-5-mini to convert the operator's instruction into a structured JSON task containing:

Drone ID

Destination

Battery consumption per cell

The LLM is instructed to return only the required JSON structure.

The parser then normalises and validates the returned values. If the OpenAI API is unavailable, a regex-based fallback parser is used so the system can continue operating.

The LLM only proposes the task; it does not decide whether the drone is allowed to execute it. Verification happens separately.

3. A\* Route Planning

Once the task has been parsed, A\* calculates a route from the drone's current position to the selected destination.

The station is represented as a 10×10 grid with:

Normal cells — cost 1× battery

Grey/toll cells — cost 3× battery

Obstacles — impassable

A\* uses a Manhattan-distance heuristic and four-directional movement. Its route cost incorporates the additional battery cost of grey cells, allowing it to account for terrain energy consumption when selecting a route.

Other parked drones are temporarily treated as obstacles during route planning, while the destination cells remain accessible.

4. Formal Safety Verification with Z3

After A\* generates a route, the system passes the proposed route to the Z3 SMT solver.

For every step of the route, Z3 represents the battery level using:

Batteryi = Battery(i-1) − Drain × Cell Cost

with the safety condition:

Battery_i ≥ 0

for every step.

The verifier checks two conditions:

A valid route exists.

The drone's battery never becomes negative during that route.

If the constraints are satisfiable, the plan is marked verified. If they are not, the plan is rejected and the drone does not move.

This creates a separation between AI-based decision making and formal safety enforcement.

5. Execution and Fail-Safe Behaviour

The system follows a verify-before-execute architecture.

A proposed trip is first parsed, planned, and verified. Only if the plan is verified can the deliver() function update the drone's position and battery. Rejected plans leave the drone stationary.

The dashboard provides two operations:

Verify Plan — calculates and verifies the route without moving the drone.

Drive — verifies the route and executes it only if verification succeeds.

6. Perception and State Estimation

The system simulates imperfect drone perception. Instead of relying on perfect sensor measurements, it generates noisy battery and position readings.

A lightweight 1D Kalman filter is used to estimate:

Battery level

X-position

Y-position

The dashboard displays the resulting True → Noisy → Estimated state.

7. Database and Decision Logging

The system uses SQLite as its persistent memory layer.

It stores:

Drones — current battery, position, status, and destination

Tasks — operator commands and their verification/execution status

Events — timestamped system events and decisions

The system records events such as task parsing, verification success/failure, and trip completion, creating an auditable decision history.

8. Overall Architecture

The implemented architecture can be summarised as:

Natural Language
↓
LLM → Structured JSON Task
↓
A\* → Terrain-aware Route
↓
Z3 → Formal Battery Safety Verification
↓
Verified?
↙ ↘
No Yes
↓ ↓
Reject Execute
↓
SQLite + Decision Log

The central design principle is that the LLM and path planner can propose actions, but formal verification controls whether those actions can actually be executed.

9. Future Goals

The next stage would extend the current routing system toward more adaptive decision-making:

RL-based multi-objective optimisation balancing travel time, battery consumption, and congestion.

Dynamic re-planning when road or drone conditions change.

Charging-aware routing incorporating charging_pad availability and charging decisions.

Multi-drone coordination for conflicts and shared road resources (future extension).

These would extend the existing A\* + Z3 architecture rather than replacing its formal safety layer.
