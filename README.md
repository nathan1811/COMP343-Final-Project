# Autonomous Drone Routing System

GitHub Repository: https://github.com/nathan1811/COMP343-Final-Project

## Demo

![Autonomous Drone Routing System Demo](demo.gif)

## 1. System Overview

The **Autonomous Drone Routing System** is an autonomous routing framework that combines natural-language task interpretation, A\* pathfinding, adaptive movement decisions, formal safety verification using Z3/SMT, perception and state estimation, execution, and SQLite decision logging.

The main pipeline is:

```text
Operator Command
       ↓
LLM / Fallback Parser
       ↓
Structured Drone Tasks
       ↓
A* Route Planning
       ↓
Adaptive Pilot
       ↓
Time-Aware Multi-Drone Scheduling
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

The LLM proposes the task, A\* proposes the spatial route, the Adaptive Pilot provides advisory movement decisions, and Z3 acts as the formal safety gate before execution.

---

## 2. Current System Working

### Step 1 — Natural-language command

The operator can enter a command such as:

> Take Drone-03 to the supermarket and Drone-04 to the airport and Drone-01 to the airport and Drone-02 to the airport. Reach by 11:00 PM.

The parser identifies every explicitly named drone, its destination, and the requested deadline.

For multi-drone commands, the parser creates a separate task for each explicitly named drone.

If the OpenAI API is unavailable, the parser uses the built-in regex fallback for supported command structures.

### Step 2 — Route planning with A\*

For each drone, A\* calculates a route through the station grid.

The environment uses:

- A 10 × 10 grid
- Four-directional movement
- Blocked/obstacle cells
- Normal terrain
- Grey/toll terrain with higher movement cost

A\* uses the Manhattan-distance heuristic and terrain costs when selecting a route.

The planner uses a conservative battery bound so that the route can be checked before execution.

### Step 3 — Battery model

The simulated drone movement uses a random actual battery drain of approximately **1.5%–3.0% per movement step**.

For planning and safety verification, the system uses **3.0% as the worst-case movement-drain bound**.

Each movement step represents **15 minutes** of simulated time.

A reserve battery level of **15%** is maintained when calculating charging requirements.

### Step 4 — Adaptive Pilot

The system includes a lightweight **Adaptive Pilot** that provides an advisory movement decision alongside the planned A\* route.

The pilot uses a tabular Q-learning-style state/action structure. Its state representation considers factors such as:

- Terrain type
- Local congestion
- Battery band

The available movement decisions are:

- **PROCEED**
- **CAUTIOUS_WAIT**

The pilot is deliberately limited in authority. It can advise a short wait before a movement, but it cannot override A\*, remove a safety constraint, or bypass Z3 verification.

The formal verification layer therefore remains the final safety gate.

### Step 5 — Automatic charging

Charging is handled automatically when a drone does not have enough battery to safely reach its destination while maintaining the required reserve.

The system first checks whether the drone can reach the destination directly. If it cannot, it searches for a suitable charging pad and plans:

```text
Drone → Charging Pad → Destination
```

The required charge level is calculated from the remaining route energy requirement plus the reserve battery percentage.

If the required charging target would exceed 100%, the plan is rejected.

Charging time is calculated from the required energy and the charging-pad power.

The operator does not need to manually specify a charging percentage for the natural-language workflow.

### Step 6 — Multi-drone coordination

When multiple drones are sent at the same time, the system creates a time-aware schedule before execution.

The collision rules are:

1. Two drones cannot occupy the **same tile at the same time**.
2. Two drones cannot use the **same charging pad at the same time**.
3. Drones may use the same route at different times.
4. A drone may pass through another drone's previous path after that drone has cleared the cell.
5. There is no additional edge-swap restriction.

If two drones would conflict, the scheduling layer can insert waiting time while preserving the spatial route.

The scheduling order uses starting battery as the priority signal, with the lower-starting-battery drone receiving priority when a conflict requires one drone to wait.

### Step 7 — Z3 / SMT verification

The proposed route and time-expanded schedule are passed to the Z3 SMT verifier.

For battery safety, the verifier models:

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

### Step 8 — Execution

The project follows a **verify-before-execute** design.

- **Verify Plan** calculates and verifies the route without moving the drone.
- **Deliver** verifies the plan and only then executes it.

A failed verification therefore leaves the affected drone stationary.

### Step 9 — Perception and state estimation

The system simulates imperfect perception by generating noisy measurements of:

- Battery level
- X position
- Y position

A lightweight 1D Kalman filter estimates the underlying state from these noisy observations.

The dashboard displays:

```text
True State → Noisy Measurement → Estimated State
```

### Step 10 — Database and decision logging

SQLite is used as the persistent memory and event-log layer.

The database records information about:

- Drones
- Current battery and position
- Status and destination
- Operator tasks
- Verification results
- Execution results
- Timestamped system events

This creates an auditable record of system decisions and execution history.

---

## 3. Default Demonstration

The Streamlit dashboard opens with the following default multi-drone command:

```text
Take Drone-03 to the supermarket and Drone-04 to the airport and Drone-01 to the airport and Drone-02 to the airport. Reach by 11:00 PM.
```

The default map seed is:

```text
8695665
```

Another map seed to try:

```text
9385349
```

The demonstration therefore starts with four drones assigned to destinations, with Drone-03 going to the supermarket and the other three drones going to the airport, with an 11:00 PM deadline.

---

## 4. System Architecture

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

The architecture deliberately separates:

- **Task interpretation**
- **Route generation**
- **Adaptive movement advice**
- **Scheduling**
- **Formal safety verification**
- **Execution**
- **Memory and logging**

The LLM is not trusted to directly control the drone. A proposed action must pass the formal verification layer before it can change the simulated drone state.

---

## 5. How to Run the Project — Windows PowerShell

Open **PowerShell** and navigate to the project folder.

### Step 1 — Go to the project directory

```powershell
cd "path\to\drone_delivery_project"
```

For example:

```powershell
cd "$HOME\Downloads\drone_delivery_project"
```

### Step 2 — Install the required packages

```powershell
pip install -r requirements.txt
```

### Step 3 — Configure the OpenAI API key

The repository includes `.env.example` as a template.

Create a local `.env` file from the template:

```powershell
Copy-Item .env.example .env
```

Open `.env` and replace:

```text
OPENAI_API_KEY=your_api_key_here
```

with your own OpenAI API key.

The `.env` file is included in `.gitignore` and must never be committed to GitHub.

The application reads the API key from the `OPENAI_API_KEY` environment variable.

### Step 4 — Start the Streamlit dashboard

```powershell
streamlit run dashboard.py
```

Streamlit will provide a local address, normally similar to:

```text
http://localhost:8501
```

Open that address in your browser if it does not open automatically.

### Complete command sequence

```powershell
cd "path\to\drone_delivery_project"
pip install -r requirements.txt
Copy-Item .env.example .env
streamlit run dashboard.py
```

Before running the application for the first time, open `.env` and add your own API key.

---

## 6. Running Without an API Key

The OpenAI API key is not strictly required for the application to run.

If no API key is available, the natural-language parser can use its regex-based fallback parser for supported command structures.

For the full GPT-5-mini natural-language workflow, set the API key before launching Streamlit.

---

## 7. Main Files

| File                | Purpose                                                                                 |
| ------------------- | --------------------------------------------------------------------------------------- |
| `models.py`         | Drone models, station grid, obstacles, terrain and charging-pad configuration           |
| `llm_parser.py`     | Natural-language command parsing, structured task generation and fallback parsing       |
| `pathfinding.py`    | Terrain-aware A\* pathfinding                                                           |
| `adaptive_pilot.py` | Lightweight adaptive movement policy                                                    |
| `verifier.py`       | Z3/SMT route, battery and multi-drone schedule verification                             |
| `orchestrator.py`   | Integrates parsing, planning, charging, scheduling, verification, execution and logging |
| `perception.py`     | Noisy sensor simulation and Kalman-based state estimation                               |
| `database.py`       | SQLite database and event logging                                                       |
| `dashboard.py`      | Streamlit user interface and live simulation                                            |
| `Documentation.md`  | Technical description of the system architecture and methodology                        |

---

## 8. Algorithmic Foundations

The project uses established algorithmic techniques from the course material as foundations for the implemented components.

The main mappings are:

- **A\*** — node representation, open/closed sets, `g/h/f` scores, priority queue, heuristic calculation, neighbour expansion and path reconstruction.
- **ReAct / structured parsing** — structured task generation, validation and bounded parser interaction.
- **Q-learning / adaptive policy** — tabular state/action representation, action selection and Q-value updates for the Adaptive Pilot.
- **SMT / Z3** — symbolic variables, hard constraints, solver execution, satisfiability checking and structured verification results.
- **State estimation** — noisy observations and Kalman-filter-based estimation.

These algorithmic foundations are integrated into the drone-routing problem while keeping the formal verification layer independent from the natural-language decision layer.

---

## 9. Key Design Principle

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
Advises movement behaviour
      ↓
Z3 / SMT
      ↓
Formally verifies safety
      ↓
Execution
```

This layered design means that individual components can propose or optimise actions without receiving unrestricted authority over execution.

The final execution decision remains subject to the formal verification layer.

---

## 10. Limitations and Safety Considerations

- The system is a simulation and does not control physical drones.
- Battery consumption is simulated rather than measured from physical hardware.
- The perception system uses simulated noisy observations.
- The LLM/parser can produce incorrect task interpretations, so its output is validated before execution.
- A\* proposes routes but does not independently guarantee complete system safety.
- The Adaptive Pilot is advisory and cannot bypass the verification layer.
- Z3/SMT verification is used as the final safety gate before simulated execution.
- The current environment models a fixed grid, limited destinations, and simulated charging pads.

---

## 11. Weights & Biases

Experiment tracking and project logs:

**W&B Project:** [Add W&B project/report link here]

---

## 12. Testing and Evaluation

The project includes testing and evaluation procedures for checking route planning, battery constraints, multi-drone scheduling, charging behaviour, and formal verification.

The evaluation files and instructions are included in the repository.
