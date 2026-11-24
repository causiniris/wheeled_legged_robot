"""Calculate and check the fixed-leg balance-car PD and speed PI starting gains."""
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm

from robot_model import upright_matrices


ROOT = Path(__file__).parent
DQ = json.loads((ROOT / "output/results.json").read_text(encoding="utf-8"))["dq"]
DT = 0.005
RAD = np.pi / 180
# Hypotheses, not a measured mass distribution: m, mw, h, Ib, Jw, Kt scale, IMU height.
NOMINAL = np.array([1.60, 0.40, 0.15, 0.012, 2 * DQ["J"], 1.0, 0.10])
DEVIATION = np.array([0.20, 0.06, 0.025, 0.004, 0.20 * NOMINAL[4], 0.15, 0.07])


def plant(parameters):
    m, mw, h, ib, jw, kt_scale, imu_height = parameters
    a, b, c, _ = upright_matrices(
        radius=0.0425, body_mass=m, rotating_mass=mw, height=h, body_inertia=ib,
        rotating_inertia=jw, dq={**DQ, "Kt": DQ["Kt"] * kt_scale}, drive_damping=DQ["B"],
    )
    # No position feedback. Remove the neutral translation coordinate.
    a, b = a[1:, 1:], b[1:, 0]
    zoh = expm(DT * np.vstack((np.column_stack((a, b)), np.zeros(5))))
    imu_accel = (a[0] + imu_height * a[2]) / 9.81
    return a, zoh[:4], c[3, 1:], imu_accel


def attitude_seed(a):
    # Quasi-static q current, v=0, target natural frequency 10 rad/s and zeta=0.27.
    gravity = a[2, 1]
    voltage_effect = -a[2, 3] / DQ["R_phase"]
    damping = -(a[2, 2] + a[2, 3] * DQ["Ke"] / DQ["R_phase"])
    kp = (gravity + 10.0**2) / voltage_effect
    kd = (2 * 0.27 * 10.0 - damping) / voltage_effect
    return (round(kp, 2), round(kd, 2)), (gravity, voltage_effect, damping)


def tick(state, system, settings, *, limited, lean_limit=3.0):
    _, zoh, relative_speed, imu_accel = system
    kp, kd, speed_kp, speed_ki, tilt_tf = settings
    pitch, shaft_speed, speed, integral, held_voltage = state[4:]
    x = zoh @ np.r_[state[:4], held_voltage]
    # Forward linear acceleration biases the gravity-only pitch measurement.
    accel_pitch = x[1] - imu_accel @ x
    pitch = (tilt_tf * (pitch + DT * x[2]) + DT * accel_pitch) / (tilt_tf + DT)
    shaft_speed += DT / (0.02 + DT) * (relative_speed @ x - shaft_speed)
    speed += DT / (0.10 + DT) * (shaft_speed - speed)
    candidate = np.clip(integral + DT * speed, -100, 100) if limited else integral + DT * speed
    requested_lean = -(speed_kp * speed + speed_ki * candidate)
    if not limited or abs(requested_lean) <= lean_limit or requested_lean * speed > 0:
        integral = candidate
    lean = -(speed_kp * speed + speed_ki * integral)
    if limited:
        lean = np.clip(lean, -lean_limit, lean_limit)
    voltage = kp * (pitch - RAD * lean) + kd * x[2]
    if limited:
        voltage = np.clip(voltage, -3, 3)
    return np.r_[x, pitch, shaft_speed, speed, integral, voltage]


def pole_radius(system, settings):
    transition = np.column_stack([tick(z, system, settings, limited=False) for z in np.eye(9)])
    poles = np.linalg.eigvals(transition)
    if settings[2] == settings[3] == 0:
        poles = np.delete(poles, np.argmin(abs(poles - 1)))  # disconnected speed integral
    return max(abs(poles))


def release(system, settings, lean_deg, *, lean_limit=3.0):
    state = np.zeros(9)
    state[1] = state[4] = lean_deg * RAD
    peak_v = peak_shaft = peak_tilt = 0.0
    for k in range(2000):
        state = tick(state, system, settings, limited=True, lean_limit=lean_limit)
        peak_v = max(peak_v, abs(state[-1]))
        peak_tilt = max(peak_tilt, abs(state[1] / RAD))
        peak_shaft = max(peak_shaft, abs(system[2] @ state[:4]))
        if peak_tilt >= 35 or peak_shaft > 30:
            break
    return ((k + 1) * DT, state[1] / RAD, state[0], peak_tilt, peak_shaft, peak_v)


if __name__ == "__main__":
    nominal = plant(NOMINAL)
    (kp, kd), coefficients = attitude_seed(nominal[0])
    old = (8.8, 0.29, 0.0, 0.0, 0.245)
    previous = (kp, kd, 0.35, 0.22, 1.30)
    revised = (56.0, 1.90, 0.31, 0.20, 1.75)
    revised_lean_limit = 1.0
    print("Reduced pitch coefficients (a, b, c):", coefficients)
    print("Revised P, D, speed P, speed I, tilt filter (s):", revised)
    print("Dominant discrete pole modulus (old, previous, revised):",
          *(pole_radius(nominal, gains) for gains in (old, previous, revised)))
    print("Release columns: time(s), final angle(deg), final speed(m/s), peak angle(deg), peak shaft(rad/s), peak Uq(V)")
    for lean in (1, 2, -2, 3):
        print("Release", lean, "deg: previous", release(nominal, previous, lean),
              "revised", release(nominal, revised, lean, lean_limit=revised_lean_limit))
    rng = np.random.default_rng(1124)
    varied = NOMINAL + rng.uniform(-1, 1, (1024, 7)) * DEVIATION
    systems = [plant(p) for p in varied]
    for name, gains in (("previous", previous), ("revised", revised)):
        radii = np.array([pole_radius(system, gains) for system in systems])
        print(f"{name} sensitivity: {np.count_nonzero(radii < 1)}/{len(radii)} sampled linear models stable, worst |pole| {max(radii):.6f}")
