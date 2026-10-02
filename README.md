# Autonomous Drone Routing System

## 1. System Overview

The **Autonomous Drone Routing System** is an autonomous routing framework that combines natural-language task interpretation, A\* pathfinding, formal safety verification using Z3/SMT, perception and state estimation, execution, and SQLite decision logging.

The main pipeline is:

```text
Operator Command
       ↓
LLM / Fallback Parser
       ↓
Structured Drone Task
       ↓
A* Route Planning
       ↓
Z3 / SMT Verification
       ↓
Verified?
   ↙       ↘
 No         Yes
 ↓           ↓
Reject     Execute
             ↓
       SQLite + Decision Log
```

The project supports both **natural-language input** and **manual input** through the Streamlit dashboard.

The LLM proposes the task, A\* proposes the route, and Z3 acts as the formal safety gate before execution.

---

## 2. Current System Working

### Step 1 — Natural-language command

The operator enters a command such as:

> Take Drone-03 to the airport and Drone-04 to the airport and Drone-01 to the airport and Drone-02 to the airport. Reach by 11:00 PM.

The parser identifies every explicitly named drone, its destination, and the requested deadline.

For multi-drone commands, the parser creates a separate task for each named drone rather than dropping any of the explicitly mentioned drones.

If the OpenAI API is unavailable, the parser uses the built-in regex fallback so that the natural-language interface can still operate.

### Step 2 — Route planning with A\*

For each drone, A\* calculates a route through the station grid.

The environment uses:

- A 10 × 10 grid
- Four-directional movement
- Blocked/obstacle cells
- Normal terrain
- Grey/toll terrain with higher movement cost

A\* uses the Manhattan-distance heuristic and terrain costs when selecting a route.

### Step 3 — Battery model

The simulated drone movement uses a random actual battery drain of approximately **1.5%–3.0% per movement step**.

For planning and safety verification, the system uses **3.0% as the worst-case movement-drain bound**. This provides a conservative safety check while the execution simulation can vary between 1.5% and 3.0%.

Each movement step represents **15 minutes** of simulated time.

### Step 4 — Automatic charging

Charging is handled automatically by the system when a drone does not have enough battery to safely reach its destination.

The system first checks whether the drone can reach the destination directly. If it cannot, it searches for a suitable charging pad and plans:

```text
Drone → Charging Pad → Destination
```

The required charge level is calculated from the remaining route energy requirement plus the configured reserve battery percentage. The system rejects a charging plan if the required target would exceed 100% battery.

Charging time is calculated from the amount of energy required and the charging-pad power.

The operator does not need to manually specify a charge percentage.

### Step 5 — Multi-drone coordination

When multiple drones are sent at the same time, the system creates a time-aware schedule before execution.

The collision rules are:

1. Two drones cannot occupy the **same tile at the same time**.
2. Two drones cannot use the **same charging pad at the same time**.
3. Drones are allowed to use the same route at different times.
4. A drone may pass through another drone's previous path after that drone has cleared the cell.
5. There is no additional edge-swap restriction.

If two drones would conflict, the drone with the **higher starting battery** waits in its previous tile while the lower-starting-battery drone gets priority and clears the conflicting location.

The schedule is then formally checked before execution.

### Step 6 — Z3 / SMT verification

The proposed route and schedule are passed to the Z3 SMT verifier.

For battery safety, the verifier models the battery evolution as:

```text
Battery(i) = Battery(i-1) - Drain × CellCost
```

and requires:

```text
Battery(i) ≥ 0
```

for every relevant movement step.

For multi-drone execution, the verifier also checks the time-expanded schedule so that conflicting drones are not assigned to the same tile at the same time and shared charging resources are not simultaneously occupied.

If the constraints are satisfiable, the plan is marked **verified**. If verification fails, the system does not execute the proposed movement.

### Step 7 — Execution

The project follows a **verify-before-execute** design.

- **Verify Plan** calculates and verifies the route without moving the drone.
- **Drive** verifies the plan and only then executes it.

A failed verification therefore leaves the affected drone stationary.

### Step 8 — Perception and state estimation

The system simulates imperfect perception by generating noisy measurements of:

- Battery level
- X position
- Y position

A lightweight 1D Kalman filter estimates the underlying state from these noisy observations.

The dashboard displays the relationship between:

```text
True State → Noisy Measurement → Estimated State
```

### Step 9 — Database and decision logging

SQLite is used as the persistent memory and event-log layer.

The database records information about:

- Drones
- Current battery and position
- Status and destination
- Operator tasks
- Verification results
- Execution results
- Timestamped system events

This creates an auditable record of the system's decisions and execution history.

---

## 3. Natural-Language Demo Prompt

Use the following prompt in the Streamlit natural-language input box to demonstrate the multi-drone system:

```text
Take Drone-03 to the airport and Drone-04 to the airport and Drone-01 to the airport and Drone-02 to the airport. Reach by 11:00 PM.
```

This prompt intentionally names **four drones** and gives them the same destination and deadline. The parser should preserve all four drone IDs:

- Drone-03 → Airport
- Drone-04 → Airport
- Drone-01 → Airport
- Drone-02 → Airport

The deadline is **11:00 PM**.

---

## 4. How to Run the Project — Windows PowerShell

Open **PowerShell** and navigate to the project folder.

### Step 1 — Go to the project directory

If the project folder is already open in your terminal, you can skip this step.

Otherwise:

```powershell
cd "path\to\drone_delivery_project"
```

For example, if the project is inside your Downloads folder:

```powershell
cd "$HOME\Downloads\drone_delivery_project"
```

### Step 2 — Install the required packages

Run:

```powershell
pip install -r requirements.txt
```

Wait for the installation to finish before continuing.

### Step 3 — Set the OpenAI API key

Before starting Streamlit, run:

```powershell
$env:OPENAI_API_KEY=""
```

Then put your actual OpenAI API key between the quotation marks. For example:

```powershell
$env:OPENAI_API_KEY="YOUR_API_KEY_HERE"
```

Do **not** commit or share your real API key in the project files or Git repository.

This environment variable applies to the current PowerShell session.

### Step 4 — Start the Streamlit dashboard

Run:

```powershell
streamlit run dashboard.py
```

Streamlit will start the application and provide a local address, normally similar to:

```text
Local URL: http://localhost:8501
```

Open that address in your browser if it does not open automatically.

### Complete command sequence

For a fresh terminal session, the complete sequence is:

```powershell
cd "path\to\drone_delivery_project"
pip install -r requirements.txt
$env:OPENAI_API_KEY="YOUR_API_KEY_HERE"
streamlit run dashboard.py
```

---

## 5. Running Without an API Key

The OpenAI API key is not strictly required for the application to run.

If no API key is available, the natural-language parser can use its regex-based fallback parser for supported command structures.

However, for the full **GPT-5-mini natural-language demonstration**, set the API key before launching Streamlit:

```powershell
$env:OPENAI_API_KEY="YOUR_API_KEY_HERE"
```

---

## 6. Main Files

| File                  | Purpose                                                                                 |
| --------------------- | --------------------------------------------------------------------------------------- |
| `models.py`           | Drone models, station grid, obstacles, terrain and charging-pad configuration           |
| `llm_parser.py`       | Natural-language command parsing, structured task generation and fallback parsing       |
| `pathfinding.py`      | Lab 1-derived terrain-aware A\* pathfinding                                             |
| `verifier.py`         | Z3/SMT route, battery and multi-drone schedule verification                             |
| `orchestrator.py`     | Integrates parsing, planning, charging, scheduling, verification, execution and logging |
| `perception.py`       | Noisy sensor simulation and Kalman-based state estimation                               |
| `database.py`         | SQLite database and event logging                                                       |
| `dashboard.py`        | Streamlit user interface                                                                |
| `CLASS_CODE_REUSE.md` | Mapping between the original class/lab implementations and the project                  |
| `Documentation.md`    | Technical description of the system architecture and methodology                        |

---

## 7. Class-Code Reuse

The project preserves the main class algorithms and adapts them to the drone-routing problem.

- **Lab 1 `planner.py` → `pathfinding.py`**: Node representation, A\* open/closed sets, `g/h/f` scores, heap queue, heuristic calculation, neighbour expansion and parent-based path reconstruction.
- **ReAct Lab → `llm_parser.py`**: Structured JSON parsing, action validation, Pydantic argument validation, observation feedback and bounded reasoning loop.
- **SMT/Z3 Lab → `verifier.py`**: Typed verification payloads, symbolic variables, hard constraints, solver execution, SAT/UNSAT handling and structured verification results.

The system keeps the A\* + Z3 architecture as the core planning and safety mechanism.

---

## 8. Key Design Principle

The system separates **proposal** from **execution**:

```text
LLM
 ↓
Proposes structured task
 ↓
A*
 ↓
Proposes route
 ↓
Z3
 ↓
Formally verifies safety
 ↓
Execution
```

The LLM is therefore not trusted to directly control the drone. A proposed action must pass the formal verification layer before it can change the simulated drone state.

---
