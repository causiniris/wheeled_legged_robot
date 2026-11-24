"""Illustrative speed-reference simulation: no real-robot calibration."""
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from model import Parameters, derivative
from cascade_model import SpeedController


def run():
    p = Parameters()
    # These are demonstration gains for hypothetical parameters, not firmware settings.
    speed = SpeedController(kp=0.03, ki=0.005, limit=np.deg2rad(3))
    kp, kd, control_dt, sub_dt = 30.0, 1.0, 0.005, 0.001
    state, filtered_velocity = np.zeros(4), 0.0
    rows = []
    for k in range(4001):
        t = k * control_dt
        reference = 0.05 if 1 <= t < 12 else 0.0
        # The plant provides ground speed directly. This is not the firmware's
        # relative encoder velocity; sensing/IMU delay is not simulated.
        filtered_velocity += control_dt / (0.1 + control_dt) * (state[1] - filtered_velocity)
        theta_ref = speed.update(reference, filtered_velocity, control_dt)
        voltage = float(np.clip(kp * (state[2] - theta_ref) + kd * state[3], -3, 3))
        rows.append([t, reference, *state, filtered_velocity, theta_ref, voltage])
        if abs(state[2]) >= np.deg2rad(35):
            break
        for _ in range(5):
            k1 = derivative(state, voltage, p)
            k2 = derivative(state + sub_dt*k1/2, voltage, p)
            k3 = derivative(state + sub_dt*k2/2, voltage, p)
            k4 = derivative(state + sub_dt*k3, voltage, p)
            state += sub_dt*(k1 + 2*k2 + 2*k3 + k4)/6
    return np.asarray(rows)


def main():
    out = Path(__file__).resolve().parent / 'output'
    out.mkdir(exist_ok=True)
    rows = run()
    with (out / 'cascade_speed_response.csv').open('w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time_s', 'velocity_reference_m_s', 'x_m', 'velocity_m_s',
                         'pitch_rad', 'pitch_rate_rad_s', 'filtered_velocity_m_s',
                         'pitch_reference_rad', 'voltage_V'])
        writer.writerows(rows)
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(rows[:, 0], rows[:, 1], '--', label='Speed reference')
    axes[0].plot(rows[:, 0], rows[:, 3], label='Ground speed')
    axes[1].plot(rows[:, 0], np.rad2deg(rows[:, 4]), label='Pitch')
    axes[1].plot(rows[:, 0], np.rad2deg(rows[:, 7]), '--', label='Pitch reference')
    axes[2].plot(rows[:, 0], rows[:, 8], label='Uq voltage')
    for ax, label in zip(axes, ['Speed (m/s)', 'Pitch (deg)', 'Uq (V)']):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_title('Hypothetical model: not identified or hardware validated')
    axes[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    fig.savefig(out / 'cascade_speed_response.png', dpi=160)
    plt.close(fig)
    print(f'duration={rows[-1,0]:.3f}s peak_pitch={max(abs(np.rad2deg(rows[:,4]))):.5f}deg '
          f'peak_Uq={max(abs(rows[:,8])):.5f}V final_speed={rows[-1,3]:.6f}m/s')
    for t in [5, 11.995, 19.995]:
        row = rows[np.argmin(abs(rows[:, 0] - t))]
        print(f't={row[0]:.3f}s vref={row[1]:.5f} v={row[3]:.5f} pitch_deg={np.rad2deg(row[4]):.5f}')


if __name__ == '__main__':
    main()
