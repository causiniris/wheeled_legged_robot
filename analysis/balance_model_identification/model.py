"""Provisional fixed-leg two-wheel inverted-pendulum and voltage-drive model."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Parameters:
    # Mechanical values are hypothetical; only 85 mm wheel diameter is a CAD name.
    radius: float = 0.0425
    body_mass: float = 1.6
    wheel_mass: float = 0.4  # total, both rotating wheels; excludes motor stators
    com_height: float = 0.15
    body_inertia: float = 0.012  # kg m^2 about body COM and pitch axis
    wheel_inertia: float = 0.00036125  # total, solid-disk assumption
    gravity: float = 9.81
    # Tentative dq convention from chapter 5, NOT verified for this drive.
    phase_resistance: float = 3.1325
    kt: float = 0.0375855
    ke: float = 0.025057


def derivative(state, voltage, p):
    _, velocity, theta, rate = state
    m, h, r = p.body_mass, p.com_height, p.radius
    mass_matrix = np.array([[m + p.wheel_mass + p.wheel_inertia / r**2,
                             m * h * np.cos(theta)],
                            [m * h * np.cos(theta), p.body_inertia + m * h**2]])
    relative_speed = velocity / r - rate
    torque = 2 * p.kt / p.phase_resistance * (voltage - p.ke * relative_speed)
    rhs = np.array([torque / r + m * h * np.sin(theta) * rate**2,
                    m * p.gravity * h * np.sin(theta) - torque])
    acceleration = np.linalg.solve(mass_matrix, rhs)
    return np.array([velocity, acceleration[0], rate, acceleration[1]])


def linearize(p):
    r, m, h = p.radius, p.body_mass, p.com_height
    matrix = np.array([[m + p.wheel_mass + p.wheel_inertia / r**2, m * h],
                       [m * h, p.body_inertia + m * h**2]])
    torque_vector = np.linalg.solve(matrix, np.array([1 / r, -1]))
    gravity_vector = np.linalg.solve(matrix, np.array([0, m * p.gravity * h]))
    torque_per_volt = 2 * p.kt / p.phase_resistance
    a = np.zeros((4, 4))
    a[0, 1] = a[2, 3] = 1
    a[[1, 3], 2] = gravity_vector
    a[[1, 3], 1] = -torque_vector * torque_per_volt * p.ke / r
    a[[1, 3], 3] = torque_vector * torque_per_volt * p.ke
    b = np.array([0, torque_vector[0] * torque_per_volt,
                  0, torque_vector[1] * torque_per_volt])
    return a, b
