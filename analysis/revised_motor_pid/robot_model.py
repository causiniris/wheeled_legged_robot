"""Parametric longitudinal model for a rigid, symmetric two-wheel robot."""
import numpy as np


def upright_matrices(*, radius, body_mass, rotating_mass, height, body_inertia,
                     rotating_inertia, dq, drive_damping):
    """Return A,B,C,D in SI units about upright and stationary.

    State: [axle position, ground velocity, forward pitch, pitch rate, mean iq].
    Input: signed common Uq. Outputs: [ground velocity, pitch, pitch rate,
    signed mean shaft speed relative to the chassis].
    rotating_mass and rotating_inertia are summed across both sides.
    drive_damping is per side, with respect to speed relative to the chassis.
    """
    r, m, h = radius, body_mass, height
    resistance, inductance, ke, kt = (dq[k] for k in ("R_phase", "L_phase", "Ke", "Kt"))
    mass = np.array([[m + rotating_mass + rotating_inertia/r**2, m*h],
                     [m*h, body_inertia + m*h**2]])
    grav = np.linalg.solve(mass, [0.0, m*9.81*h])
    torque = np.linalg.solve(mass, [1/r, -1])
    relative_speed = np.array([0.0, 1/r, 0.0, -1.0, 0.0])
    a = np.zeros((5, 5))
    a[0, 1] = a[2, 3] = 1.0
    a[[1, 3], :] = -2*drive_damping*np.outer(torque, relative_speed)
    a[[1, 3], 2] += grav
    a[[1, 3], 4] = 2*kt*torque
    a[4, :] = -(ke/inductance)*relative_speed
    a[4, 4] = -resistance/inductance
    b = np.array([[0.0], [0.0], [0.0], [0.0], [1/inductance]])
    c = np.vstack((np.eye(5)[[1, 2, 3]], relative_speed))
    return a, b, c, np.zeros((4, 1))
