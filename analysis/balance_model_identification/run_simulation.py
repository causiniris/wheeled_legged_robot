"""Run an illustrative simulation, not a calibration or hardware controller."""
from dataclasses import replace
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from model import Parameters, derivative, linearize

OUT = Path(__file__).resolve().parent / 'output'
DT, CONTROL_PERIOD, DURATION = 0.001, 0.005, 5.0
LIMIT, STOP_ANGLE = 3.0, np.deg2rad(35)


def simulate(p, controller):
    state = np.array([0.0, 0.0, np.deg2rad(2), 0.0])
    result, command = [], 0.0
    for n in range(round(DURATION / DT) + 1):
        t = n * DT
        if n % round(CONTROL_PERIOD / DT) == 0:
            command = np.clip(controller(state), -LIMIT, LIMIT)
        result.append([t, *state, command])
        if abs(state[2]) >= STOP_ANGLE:
            break
        k1 = derivative(state, command, p)
        k2 = derivative(state + DT * k1 / 2, command, p)
        k3 = derivative(state + DT * k2 / 2, command, p)
        k4 = derivative(state + DT * k3, command, p)
        state = state + DT * (k1 + 2*k2 + 2*k3 + k4) / 6
    return np.array(result)


def main():
    OUT.mkdir(exist_ok=True)
    p = Parameters()
    cases = {
        'open_loop': lambda z: 0.0,
        'firmware_PD': lambda z: 8.8*z[2] + 0.29*z[3],
        'illustrative_PD_plus_speed': lambda z: 30*z[2] + z[3] + z[1],
    }
    outputs = {name: simulate(p, law) for name, law in cases.items()}
    with (OUT / 'simulation.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['case', 'time_s', 'x_m', 'velocity_m_s', 'pitch_rad',
                         'pitch_rate_rad_s', 'voltage_V'])
        for name, data in outputs.items():
            for row in data[::5]:
                writer.writerow([name, *row])
    fig, axes = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    for name, data in outputs.items():
        axes[0].plot(data[:, 0], np.rad2deg(data[:, 3]), label=name)
        axes[1].plot(data[:, 0], data[:, 2], label=name)
        axes[2].plot(data[:, 0], data[:, 5], label=name)
    for ax, label in zip(axes, ['Pitch (deg)', 'Wheel-ground speed (m/s)', 'Uq command (V)']):
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[0].set_title('Illustrative assumptions only: not identified or hardware validated')
    axes[-1].set_xlabel('Time (s)')
    fig.tight_layout()
    fig.savefig(OUT / 'illustrative_response.png', dpi=160)
    plt.close(fig)
    a, b = linearize(p)
    lines = ['ILLUSTRATIVE MODEL, NOT IDENTIFIED OR VALIDATED',
             'State [x m, xdot m/s, pitch rad, pitch_rate rad/s]',
             'A =\n' + np.array2string(a, precision=6),
             'B = ' + np.array2string(b, precision=6),
             'Open-loop eigenvalues = ' + str(np.linalg.eigvals(a))]
    for name, data in outputs.items():
        lines.append(f'{name}: duration={data[-1,0]:.3f}s, '
                     f'final_pitch={np.rad2deg(data[-1,3]):.4f}deg, '
                     f'peak_abs_Uq={max(abs(data[:,5])):.4f}V, '
                     f'final_speed={data[-1,2]:.4f}m/s')
    lines.append('COM height sensitivity, illustrative PD+speed, non-position eigenvalues:')
    for h in [0.10, 0.15, 0.20, 0.25]:
        aa, bb = linearize(replace(p, com_height=h))
        eig = np.linalg.eigvals(aa + np.outer(bb, [0, 1, 30, 1]))
        lines.append(f'h={h:.2f}m: {eig}')
    report = '\n'.join(lines) + '\n'
    (OUT / 'numerical_summary.txt').write_text(report, encoding='utf-8')
    print(report)


if __name__ == '__main__':
    main()
