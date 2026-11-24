"""Illustrative speed PI -> attitude PD; not calibrated for hardware."""
from dataclasses import dataclass
import numpy as np


@dataclass
class SpeedController:
    kp: float
    ki: float
    limit: float
    integral: float = 0.0

    def update(self, reference, measured, dt):
        error = reference - measured
        trial_integral = self.integral + error * dt
        trial = self.kp * error + self.ki * trial_integral
        if abs(trial) <= self.limit or trial * error < 0:
            self.integral = trial_integral
        return float(np.clip(self.kp * error + self.ki * self.integral,
                             -self.limit, self.limit))
