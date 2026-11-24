"""Fit left wheel bench data and reproduce the classical PID calculations."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import brentq, least_squares


def read_record(path, command):
    data = np.genfromtxt(path, delimiter=",", names=True)
    t = data["t_us"] * 1e-6
    assert np.all((np.diff(t) > 0) & (np.diff(t) <= 0.001000001)), path
    step = np.flatnonzero(abs(data[command] - data[command][0]) > 1e-6)[0]
    return t - t[step], data, step


def predict(t, voltage, params):
    gain, slow, *fast = params
    first = -np.expm1(-t / slow)
    if not fast:
        return voltage * gain * first
    fast = fast[0]
    if abs(slow - fast) < 1e-6 * max(slow, fast):
        ratio = t / ((slow + fast) / 2)
        return voltage * gain * (-np.expm1(-ratio) - ratio * np.exp(-ratio))
    return voltage * gain * (slow * first + fast * np.expm1(-t / fast)) / (slow - fast)


def fit_models(record):
    t, u, measured = record
    guess = [np.mean(measured[-1000:]) / u, 0.8]
    fits = []
    for order, initial in [(1, guess), (2, guess + [0.02])]:
        result = least_squares(
            lambda log_params: predict(t, u, np.exp(log_params)) - measured,
            np.log(initial), bounds=(np.log([1e-3] + [1e-7] * order),
                                     np.log([1e3] + [30] * order)),
        )
        assert result.success, result.message
        gain, *times = np.exp(result.x)
        fits.append([gain, *sorted(times, reverse=True)])
    return fits


def physical(gain, time, resistance, inductance, ke, kt):
    slow = 1 / time
    damping = (kt / gain - ke * kt) / resistance
    inertia = (damping + ke * kt / (resistance - inductance * slow)) / slow
    fast = resistance / inductance + damping / inertia - slow
    return damping, inertia, fast


def design(gain, time, feedback_tf, derivative_tf, rate=2.0):
    # Choose a repeated dominant real pole. The fixed second-leading coefficient sets
    # the sum of the remaining poles. With PIDF choose one fast pole at -40.
    pi_base = np.polymul([1, 0], np.polymul([time, 1], [feedback_tf, 1]))
    pi_poles = -np.array([rate, rate, pi_base[1] / pi_base[0] - 2*rate])
    pi_target = pi_base[0] * np.poly(pi_poles)
    p, i = (pi_target[-2:] - pi_base[-2:]) / gain

    pid_base = np.polymul(pi_base, [derivative_tf, 1])
    pid_poles = -np.array([rate, rate, 40, pid_base[1] / pid_base[0] - 40 - 2*rate])
    pid_target = pid_base[0] * np.poly(pid_poles)
    n2, n1, ki = (pid_target[-3:] - pid_base[-3:]) / gain
    kp = n1 - derivative_tf * ki
    kd = n2 - derivative_tf * kp
    assert min(p, i, kp, ki, kd) > 0
    return (p, i, 0.0), (kp, ki, kd)


def analytical_loop(gain, times, gains, h, f):
    kp, ki, kd = gains
    n = [kp, ki] if kd == 0 else [kp*f + kd, kp + ki*f, ki]
    d = [1, 0]
    for time in [*times, h] + ([f] if kd else []):
        d = np.polymul(d, [time, 1])
    n = np.asarray(n) * gain
    poles = np.roots(np.polyadd(d, n))
    assert np.max(poles.real) < 0

    def loop(w):
        return np.polyval(n, 1j*w) / np.polyval(d, 1j*w)

    w = np.geomspace(1e-4, 1e5, 8000)
    l = loop(w)
    unity = np.flatnonzero(np.diff(np.sign(abs(l) - 1)))
    phase_crossings = np.flatnonzero(np.diff(np.sign(l.imag)))
    wc = [brentq(lambda x: abs(loop(x)) - 1, w[j], w[j+1]) for j in unity]
    wp = [brentq(lambda x: loop(x).imag, w[j], w[j+1]) for j in phase_crossings]
    return dict(poles=[[float(v.real), float(v.imag)] for v in poles],
                gain_crossovers_rad_s=wc, phase_margin_deg=min(180 + np.angle(loop(x), deg=True) for x in wc),
                gain_margin_db=min((-20*np.log10(abs(loop(x))) for x in wp if loop(x).real < 0),
                                   default=None))


def simulate(t, reference, gain, time, gains, feedback_tf, derivative_tf, input_voltage=None):
    """ZOH first-order motor, plus the firmware's end-of-sample speed filter."""
    kp, ki, kd = gains
    y = fb = integral = d_state = prior_voltage = 0.0
    output = np.empty((len(t), 3))
    for k, dt in enumerate(np.r_[0.00075, np.diff(t)]):
        y += -np.expm1(-dt / time) * (gain * prior_voltage - y)
        error = reference[k] - fb
        d_state += dt / (derivative_tf + dt) * (fb - d_state)
        base = kp * error - kd * (fb - d_state) / derivative_tf
        candidate = max(-2.0, min(2.0, integral + ki * error * dt))
        trial = base + candidate
        if abs(trial) <= 2 or trial * error < 0:
            integral = candidate
        prior_voltage = max(-2.0, min(2.0, base + integral))
        if input_voltage is not None:
            prior_voltage = input_voltage[k]
        fb += dt / (feedback_tf + dt) * (y - fb)
        output[k] = y, fb, prior_voltage
    return output


def metrics(t, y, voltage):
    take = t >= 0
    t, y, voltage = t[take], y[take], voltage[take]
    out = np.flatnonzero(abs(y - 10) > 0.2)
    index = out[-1] + 1 if len(out) else 0
    settling = float(t[index]) if index < len(t) - 1 else None
    return dict(overshoot_pct=float(max(0.0, (max(y)/10 - 1)*100)),
                settle_2pct_s=settling, observed_in_band_s=float(t[-1]-settling) if settling is not None else 0.0,
                final_second_rad_s=float(np.mean(y[t >= t[-1]-1])),
                peak_voltage_V=float(max(abs(voltage))))


def main(root, out):
    out.mkdir(parents=True, exist_ok=True)
    evidence = root / "evidence"
    records = {}
    for run in range(2, 8):
        t, data, ix = read_record(evidence / "04_left_four_levels" / f"run_{run:02d}_L_STEP.csv", "uq_cmd_V")
        u = data["uq_cmd_V"]
        assert np.allclose(u[ix:], u[ix]), run
        y = data["omega_rad_s"]
        records[run] = t[ix:], float(u[ix]-u[0]), y[ix:]-np.median(y[:ix])
    first, second = fit_models(records[2])
    gain, time = first
    winding = np.genfromtxt(root / "motor_winding_measurement.csv", delimiter=",", names=True, dtype=None, encoding="utf-8")
    values = dict(zip(winding["quantity"], winding["value"]))
    resistance, inductance = values["Ra"] / 2, values["La"] * 0.001 / 2
    ke = 7 * 0.0035
    kt = 1.5 * ke
    damping, inertia, fast = physical(gain, time, resistance, inductance, ke, kt)
    physical_model = [gain, time, 1/fast]
    result = dict(first_order=first, two_real_poles_fit=second, step_runs={},
                  dq=dict(R_phase=resistance, L_phase=inductance, Ke=ke, Kt=kt, B=damping,
                          J=inertia, fast_pole=-fast, numerator=kt/(inductance*inertia),
                          denominator=np.array([1, fast+1/time, fast/time]).tolist()))
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), layout="constrained", sharex=True)
    for (run, (t, u, y)), ax in zip(records.items(), axes.flat):
        row = {"voltage_V": u, "measurement_last_s": float(np.mean(y[t >= t[-1]-1]))}
        ax.plot(t[::5], y[::5], color="0.66", linewidth=0.8, label="Measured")
        for label, params, style in [("one_pole", first, "-"), ("two_poles", second, "--"),
                                     ("dq", physical_model, ":")]:
            yhat = predict(t, u, params)
            row[label+"_rmse"] = float(np.sqrt(np.mean((yhat-y)**2)))
            row[label+"_tail_error_pct"] = float(100 * abs(np.mean(yhat[t >= t[-1]-1]) /
                                                            row["measurement_last_s"] - 1))
            ax.plot(t[::5], yhat[::5], style, label=label)
        result["step_runs"][run] = row
        ax.set(title=f"Run {run} | {u:g} V | {'fit' if run == 2 else 'holdout'}",
               xlabel="Time after step (s)", ylabel="Speed (rad/s)")
        ax.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    fig.savefig(out / "fit_validation.png", dpi=160)
    plt.close(fig)

    # Hold out the real PI run. Actual Uq replay and closed-loop control
    # prediction are reported as separate checks.
    t, data, _ = read_record(evidence / "06_left_six_pi" / "run_14_L_PI.csv", "omega_ref_rad_s")
    reference, u = data["omega_ref_rad_s"], data["uq_cmd_V"]
    h, f = 0.020, 0.030
    baseline = (0.031, 0.043, 0.0)
    replay = simulate(t, reference, gain, time, baseline, h, f, input_voltage=u)
    predicted = simulate(t, reference, gain, time, baseline, h, f)
    real = data["omega_fb_rad_s"]
    result["hardware_PI"] = dict(measured=metrics(t, real, u), closed_loop_prediction=metrics(t, predicted[:, 1], predicted[:, 2]),
                                closed_loop_rmse=float(np.sqrt(np.mean((predicted[t >= 0, 1]-real[t >= 0])**2))),
                                voltage_replay_rmse=float(np.sqrt(np.mean((replay[t >= 0, 1]-real[t >= 0])**2))))

    pi, pid = design(gain, time, h, f)
    _, slow_pid = design(gain, time, h, f, 1.4)
    _, fast_pid = design(gain, time, h, f, 2.4)
    result["controllers"] = {}
    t_sim = np.arange(-800, 10668) * 0.00075
    ref_sim = 10.0 * (t_sim >= 0)
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex="col", layout="constrained")
    axes[0, 0].plot(t, real, color="0.55", label="Measured PI")
    axes[0, 0].plot(t, predicted[:, 1], label="Simulated PI")
    axes[0, 0].plot(t, replay[:, 1], "--", label="Recorded voltage replay")
    axes[1, 0].plot(t, u, color="0.55", label="Measured command")
    axes[1, 0].plot(t, predicted[:, 2], label="Simulated command")
    for label, gains in [("Bench PI baseline", baseline), ("Pole-matched PI", pi),
                         ("PI with PIDF P and I", (*pid[:2], 0.0)), ("PIDF A: -1.4", slow_pid),
                         ("PIDF B: -2.0", pid), ("PIDF C: -2.4", fast_pid)]:
        sim = simulate(t_sim, ref_sim, gain, time, gains, h, f)
        result["controllers"][label] = dict(gains=list(gains), simulation=metrics(t_sim, sim[:, 1], sim[:, 2]),
                                            linear=analytical_loop(gain, [time], gains, h, f),
                                            dq_linear=analytical_loop(gain, physical_model[1:], gains, h, f))
        axes[0, 1].plot(t_sim, sim[:, 1], label=label)
        axes[1, 1].plot(t_sim, sim[:, 2], label=label)
    for ax in axes[0]:
        ax.axhline(10, color="k", linestyle=":", linewidth=1)
        ax.set_ylabel("Filtered speed (rad/s)")
    for ax in axes[1]:
        ax.set(xlabel="Time after step (s)", ylabel="Uq command (V)")
    for ax in axes.flat:
        ax.legend(fontsize=8)
        ax.grid(alpha=.25)
    axes[0, 0].set_title("Independent PI hardware check")
    axes[0, 1].set_title("Bench speed controller simulation")
    fig.savefig(out / "pid_comparison.png", dpi=160)
    plt.close(fig)
    (out / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Path to new_homework")
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "output")
    args = parser.parse_args()
    main(args.source, args.output)
