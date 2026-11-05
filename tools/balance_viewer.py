"""Live balance_car_new telemetry, bounded-gain commands, and optional CSV log."""
import argparse
from collections import deque
import csv
import math
import sys
import time

FIELDS = ("ms", "pitch", "set", "rate", "wl", "wr", "u", "lean", "A", "T", "F", "kp", "ki", "kd")


def parse_sample(line):
    fields = {}
    for token in line.split():
        if "=" in token:
            key, value = token.split("=", 1)
            fields[key] = value
    if not all(key in fields for key in FIELDS):
        return None
    try:
        sample = {key: float(fields[key]) for key in FIELDS}
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in sample.values()):
        return None
    if sample["ms"] < 0 or sample["A"] not in (0, 1) or sample["F"] not in (0, 1) or sample["T"] not in (0, 1, 2):
        return None
    return sample


def main():
    parser = argparse.ArgumentParser(description="Plot and control balance_car_new")
    parser.add_argument("--port", help="ESP32 serial port, e.g. COM10")
    parser.add_argument("--seconds", type=float, default=20, help="Visible history in seconds")
    parser.add_argument("--csv", help="Save raw telemetry to a new CSV file")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    try:
        import serial
        from serial.tools import list_ports
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button
    except ImportError as exc:
        print(f"Missing dependency: {exc}. Install: python -m pip install pyserial matplotlib", file=sys.stderr)
        return 1
    port = args.port
    if not port:
        ports = list(list_ports.comports())
        if len(ports) != 1:
            print("Specify --port COM10. Available ports:", file=sys.stderr)
            for item in ports:
                print(f"  {item.device}: {item.description}", file=sys.stderr)
            return 1
        port = ports[0].device
    log = None
    if args.csv:
        try:
            log = open(args.csv, "x", newline="", encoding="utf-8")
        except OSError as exc:
            print(f"Cannot create CSV (existing files are not overwritten): {exc}", file=sys.stderr)
            return 1
    try:
        connection = serial.Serial(port, 115200, timeout=0, write_timeout=0.1)
    except (serial.SerialException, OSError) as exc:
        if log:
            log.close()
        print(f"Cannot open {port}: {exc}. Close PlatformIO Monitor first.", file=sys.stderr)
        return 1
    writer = csv.DictWriter(log, fieldnames=FIELDS) if log else None
    if writer:
        writer.writeheader()
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    fig.canvas.manager.set_window_title(f"Balance Viewer - {port}")
    fig.subplots_adjust(left=0.09, right=0.98, top=0.97, bottom=0.24, hspace=0.15)
    groups = (("pitch", "set", "target"), ("rate",), ("u",), ("wl", "wr"))
    labels = ("Pitch / set (deg)", "Pitch rate (deg/s)", "Command (V)", "Wheel (rad/s)")
    plots = {}
    for axis, keys, label in zip(axes, groups, labels):
        for key in keys:
            plots[key], = axis.plot([], [], label=key)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")
    axes[-1].set_xlabel("Time (s)")
    status = fig.text(0.02, 0.19, "Waiting for balance telemetry; startup messages are in terminal.")
    notice = fig.text(0.02, 0.155, "Gains apply only while stopped. FOC alignment moves wheels at reset.", fontsize=9)
    buttons = []

    def send(command):
        try:
            connection.write(command.encode("ascii"))
            notice.set_text(f"Sent {command}; check serial output for acceptance.")
        except (serial.SerialException, OSError) as exc:
            notice.set_text(f"Serial send failed: {exc}; cut motor power if necessary.")

    commands = (("START A", "A"), ("STOP X", "X"), ("CAL C", "C"), ("LEFT L", "L"), ("RIGHT R", "R"),
                ("Kp +", "1"), ("Kp -", "0"), ("Ki +", "5"),
                ("Ki -", "4"), ("Kd +", "3"), ("Kd -", "2"))
    for i, (title, key) in enumerate(commands):
        rect = (0.02 + (i % 6) * 0.16, 0.085 if i < 6 else 0.02, 0.14, 0.05)
        button = Button(fig.add_axes(rect), title, color="lightcoral" if key == "X" else "0.85")
        button.on_clicked(lambda event, command=key: send(command))
        buttons.append(button)
    capacity = max(100, int(args.seconds * 40))
    times = deque(maxlen=capacity)
    data = {key: deque(maxlen=capacity) for keys in groups for key in keys}
    pending = bytearray()
    origin = last_time = last_sample_wall = None

    def update(_):
        nonlocal origin, last_time, last_sample_wall
        try:
            pending.extend(connection.read(min(connection.in_waiting, 8192)))
        except (serial.SerialException, OSError) as exc:
            status.set_text(f"Serial disconnected: {exc}; cut motor power if necessary.")
            return []
        while b"\n" in pending:
            raw, _, rest = pending.partition(b"\n")
            pending[:] = rest
            line = raw.decode("utf-8", errors="replace").strip()
            sample = parse_sample(line)
            if sample is None:
                if line:
                    print(line, flush=True)
                    if "FAULT" in line or "FAILED" in line or "Cannot start" in line:
                        status.set_text(line)
                continue
            now = sample["ms"] / 1000.0
            if origin is None or (last_time is not None and now < last_time):
                origin = now
                times.clear()
                for column in data.values():
                    column.clear()
            last_time = now
            last_sample_wall = time.monotonic()
            times.append(now - origin)
            for key in data:
                value = ((sample["set"] + sample["lean"] + 180) % 360 - 180) if key == "target" else sample[key]
                data[key].append(value)
            if writer:
                writer.writerow(sample)
                log.flush()
            status.set_text(f"{port} | A={int(sample['A'])} F={int(sample['F'])} | "
                            f"Kp={sample['kp']:.2f} Ki={sample['ki']:.2f} Kd={sample['kd']:.2f}")
        if len(pending) > 16384:
            pending.clear()
        if last_sample_wall is not None and time.monotonic() - last_sample_wall > 1:
            status.set_text("Telemetry stale (>1 s): do not rely on display; check serial and cut power if needed.")
        if times:
            for key, line in plots.items():
                line.set_data(times, data[key])
            for axis in axes:
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)
                axis.set_xlim(max(0, times[-1] - args.seconds), max(args.seconds, times[-1]))
        return []

    animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    fig._balance_animation = animation
    fig._balance_buttons = buttons
    try:
        plt.show()
    finally:
        try:
            connection.write(b"X")
        except (serial.SerialException, OSError):
            pass
        connection.close()
        if log:
            log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
