"""
perception.py
--------------
Section 3: Noisy Perception and State Estimation.

The station never gets to see a drone's TRUE battery or position directly.
It only gets noisy sensor readings, and must estimate the true state from
them. This module simulates the noise and implements a small 1D Kalman
filter used for both the battery estimate and each position coordinate.
"""
import random
from dataclasses import dataclass


@dataclass
class KalmanFilter1D:
    """A minimal scalar Kalman filter: state = a single number (battery %,
    or one coordinate of position). process_var/measurement_var are tuned
    per sensor type."""
    estimate: float
    error_estimate: float
    process_var: float = 0.05
    measurement_var: float = 1.0

    def update(self, measurement: float) -> float:
        # Prediction step (no control input -- state assumed roughly static
        # between ticks, process noise accounts for drift).
        self.error_estimate += self.process_var

        # Correction step
        kalman_gain = self.error_estimate / (self.error_estimate + self.measurement_var)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error_estimate *= (1 - kalman_gain)
        return self.estimate


def noisy_battery_reading(true_battery: float, sigma: float = 1.5) -> float:
    reading = true_battery + random.gauss(0, sigma)
    return max(0.0, min(100.0, reading))


def noisy_position_reading(true_x: float, true_y: float, sigma: float = 0.4):
    return (true_x + random.gauss(0, sigma), true_y + random.gauss(0, sigma))


class DroneStateEstimator:
    """
    Bundles the Kalman filters for one drone's battery, x, and y so the
    orchestrator can call `.tick()` once per simulation step and get back
    True -> Noisy -> Estimated for the dashboard's three-column display.
    """

    def __init__(self, initial_battery: float, initial_x: float, initial_y: float):
        self.battery_filter = KalmanFilter1D(estimate=initial_battery, error_estimate=1.0,
                                              process_var=0.05, measurement_var=2.0)
        self.x_filter = KalmanFilter1D(estimate=initial_x, error_estimate=1.0,
                                        process_var=0.02, measurement_var=0.3)
        self.y_filter = KalmanFilter1D(estimate=initial_y, error_estimate=1.0,
                                        process_var=0.02, measurement_var=0.3)

    def tick(self, true_battery: float, true_x: float, true_y: float) -> dict:
        noisy_b = noisy_battery_reading(true_battery)
        noisy_x, noisy_y = noisy_position_reading(true_x, true_y)

        est_b = self.battery_filter.update(noisy_b)
        est_x = self.x_filter.update(noisy_x)
        est_y = self.y_filter.update(noisy_y)

        return {
            "true": {"battery": true_battery, "x": true_x, "y": true_y},
            "noisy": {"battery": round(noisy_b, 2), "x": round(noisy_x, 2), "y": round(noisy_y, 2)},
            "estimated": {"battery": round(est_b, 2), "x": round(est_x, 2), "y": round(est_y, 2)},
        }
