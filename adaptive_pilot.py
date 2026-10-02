from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np


N_TERRAIN_TYPES = 2          # 0=Normal, 1=Grey
N_CONGESTION_TYPES = 2       # 0=Clear, 1=Blocked
N_BATTERY_BANDS = 3          # 0=Low, 1=Medium, 2=High
N_STATES = N_TERRAIN_TYPES * N_CONGESTION_TYPES * N_BATTERY_BANDS

PROCEED = 0
CAUTIOUS_WAIT = 1
N_ACTIONS = 2
ACTION_NAMES = ["PROCEED", "CAUTIOUS_WAIT"]


def state_index(terrain: int, congestion: int, battery_band: int) -> int:
    """Map (terrain, congestion, battery_band) to a Q-table row."""
    return ((terrain * N_CONGESTION_TYPES) + congestion) * N_BATTERY_BANDS + battery_band


def index_to_state(idx: int) -> Tuple[int, int, int]:
    """Inverse of state_index()."""
    battery_band = idx % N_BATTERY_BANDS
    remainder = idx // N_BATTERY_BANDS
    congestion = remainder % N_CONGESTION_TYPES
    terrain = remainder // N_CONGESTION_TYPES
    return terrain, congestion, battery_band



def qlearning_update(Q, s, a, r, s_next, alpha, gamma):
    """
    Q-learning update rule from the class lab.

    The function does not mutate Q. It returns the new scalar value, matching
    the class lab's qlearning_update() contract.
    """
    if s_next is None:
        target = r
    else:
        target = r + gamma * np.max(Q[s_next])

    return Q[s, a] + alpha * (target - Q[s, a])


def epsilon_greedy(Q, state, epsilon, rng):
    """Choose an action using the class lab's epsilon-greedy rule."""
    s = state_index(*state)
    if rng.random() < epsilon:
        return rng.randrange(N_ACTIONS)
    return int(np.argmax(Q[s]))


def linear_epsilon_schedule(episode, total_episodes,
                            eps_start=1.0, eps_end=0.05):
    """Same linear epsilon schedule used by the class lab."""
    frac = min(1.0, episode / max(1, total_episodes * 0.8))
    return eps_start + frac * (eps_end - eps_start)


def init_Q():
    """Initialize the tabular Q matrix, as in the class lab."""
    return np.zeros((N_STATES, N_ACTIONS))


@dataclass
class PilotDecision:
    action: str
    state: Tuple[int, int, int]
    q_values: Dict[str, float]
    reason: str


class AdaptivePilot:
    """
    Drone adaptation of the class Q-learning controller.

    The learned table is deliberately small. The pilot only decides whether
    to proceed immediately or add one cautious 15-minute wait before the next
    route cell.
    """

    def __init__(self, alpha: float = 0.1, gamma: float = 0.95,
                 epsilon: float = 0.2, seed: int = 0):
        # These defaults mirror the class Q-learning lab's training values.
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.rng = random.Random(seed)
        self.Q = init_Q()
        self.training_steps = 0

    @staticmethod
    def battery_band(battery: float) -> int:
        if battery < 20.0:
            return 0       # LOW
        if battery < 50.0:
            return 1       # MEDIUM
        return 2           # HIGH

    @staticmethod
    def terrain_index(terrain: str) -> int:
        return 1 if terrain == "GREY" else 0

    @staticmethod
    def congestion_index(congestion: bool) -> int:
        return 1 if congestion else 0

    def make_state(self, terrain: str, congestion: bool,
                   battery: float) -> Tuple[int, int, int]:
        return (
            self.terrain_index(terrain),
            self.congestion_index(congestion),
            self.battery_band(battery),
        )

    def choose_action(self, terrain: str, congestion: bool,
                      battery: float, epsilon: Optional[float] = None) -> PilotDecision:
        """Use the class epsilon-greedy policy on the drone state."""
        state = self.make_state(terrain, congestion, battery)
        eps = self.epsilon if epsilon is None else epsilon
        action_index = epsilon_greedy(self.Q, state, eps, self.rng)
        action = ACTION_NAMES[action_index]

        # The policy is advisory, but we avoid deliberately adding waits in
        # ordinary cells when there is no reason to be cautious.
        if action == "CAUTIOUS_WAIT" and not congestion and terrain != "GREY":
            action = "PROCEED"
            action_index = PROCEED
            reason = "normal terrain"
        elif eps > 0 and self.rng.random() < eps:
            reason = "epsilon-greedy exploration"
        else:
            reason = "learned Q policy"

        values = self.Q[state_index(*state)]
        return PilotDecision(
            action=action,
            state=state,
            q_values={
                "PROCEED": float(values[PROCEED]),
                "CAUTIOUS_WAIT": float(values[CAUTIOUS_WAIT]),
            },
            reason=reason,
        )

    def update(self, state: Tuple[int, int, int], action: str,
               reward: float, next_state: Optional[Tuple[int, int, int]] = None) -> float:
        """Apply the class qlearning_update() and write the returned value."""
        s = state_index(*state)
        a = ACTION_NAMES.index(action)
        s_next = None if next_state is None else state_index(*next_state)
        new_value = qlearning_update(
            self.Q, s, a, reward, s_next, self.alpha, self.gamma
        )
        self.Q[s, a] = new_value
        self.training_steps += 1
        return float(new_value)

    def observe_transition(self, terrain: str, congestion: bool,
                           battery: float, action: str, reward: float,
                           next_terrain: Optional[str] = None,
                           next_congestion: bool = False,
                           next_battery: Optional[float] = None) -> float:
        """Adapt one class-style Q-learning transition to a drone state."""
        state = self.make_state(terrain, congestion, battery)
        next_state = None
        if next_terrain is not None and next_battery is not None:
            next_state = self.make_state(
                next_terrain, next_congestion, next_battery
            )
        return self.update(state, action, reward, next_state)

    def route_advice(self, route, station, battery: float,
                     congestion_by_step=None) -> List[PilotDecision]:
        """
        Apply the learned policy to an A* route without changing the route.

        This is the integration point with the existing project. The pilot
        evaluates the next cell; A* still owns the route itself.
        """
        decisions: List[PilotDecision] = []
        current_battery = battery
        congestion_by_step = congestion_by_step or []

        for index, cell in enumerate(route[1:]):
            terrain = "GREY" if station.is_grey(*cell) else "NORMAL"
            congestion = (
                bool(congestion_by_step[index])
                if index < len(congestion_by_step) else False
            )
            decision = self.choose_action(
                terrain, congestion, current_battery, epsilon=0.0
            )
            decisions.append(decision)
            current_battery -= station.cost(*cell) * 1.0

        return decisions

    def learn_from_successful_route(self, route, decisions, station) -> None:
        """
        Feed the completed route back into the same Q-learning update used in
        class. Rewards are deliberately simple and tied to the drone task:
        normal progress is positive; a cautious wait before an adverse cell is
        given a positive safety reward.
        """
        if not decisions:
            return

        for index, (cell, decision) in enumerate(zip(route[1:], decisions)):
            terrain = "GREY" if station.is_grey(*cell) else "NORMAL"

            # Successful movement is rewarded. A cautious wait is useful when
            # entering adverse terrain, while unnecessary waiting is mildly
            # discouraged so the policy does not learn to wait everywhere.
            if decision.action == "CAUTIOUS_WAIT" and terrain == "GREY":
                reward = 1.0
            elif decision.action == "CAUTIOUS_WAIT":
                reward = -0.25
            else:
                reward = 1.0 if terrain == "NORMAL" else 0.5

            next_state = None
            if index + 1 < len(decisions):
                next_cell = route[index + 2]
                next_terrain = "GREY" if station.is_grey(*next_cell) else "NORMAL"
                # Use the state stored by the following decision rather than
                # inventing another state representation.
                next_state = decisions[index + 1].state

            self.update(decision.state, decision.action, reward, next_state)

    def policy_summary(self) -> Dict[str, int]:
        """Return simple counts useful for the dashboard/log."""
        proceed = 0
        cautious = 0
        for row in self.Q:
            if int(np.argmax(row)) == PROCEED:
                proceed += 1
            else:
                cautious += 1
        return {"PROCEED": proceed, "CAUTIOUS_WAIT": cautious}
