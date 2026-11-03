"""Live plots for the sensor_viewer ESP32 firmware (115200 baud)."""
import argparse
from collections import deque
import math
import sys


def parse_sample(line):
    """Parse one sensor CSV row; ignore boot messages and malformed rows."""
    fields = line.strip().split(",")
    if len(fields) != 12:
        return None
    try:
        values = [float(field) for field in fields]
    except ValueError:
        return None
    if not math.isfinite(values[0]) or values[0] < 0:
        return None
    if any(value not in (0.0, 1.0) for value in values[9:]):
        return None
    return values


def main():
    parser = argparse.ArgumentParser(description="Plot MT6701 and MPU6050 serial data")
    parser.add_argument("--port", help="Serial port, for example COM3")
    parser.add_argument("--seconds", type=float, default=20, help="Visible history in seconds")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    try:
        import serial
        from serial.tools import list_ports
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as exc:
        print(f"Missing dependency: {exc}. Install: python -m pip install pyserial matplotlib", file=sys.stderr)
        return 1

    ports = list(list_ports.comports())
    port = args.port
    if not port:
        if len(ports) != 1:
            print("Specify the ESP32 serial port with --port COM3. Available ports:", file=sys.stderr)
            for item in ports:
                print(f"  {item.device}: {item.description}", file=sys.stderr)
            return 1
        port = ports[0].device
    try:
        connection = serial.Serial(port, 115200, timeout=0)
    except serial.SerialException as exc:
        print(f"Cannot open {port}: {exc}. Close PlatformIO Monitor first.", file=sys.stderr)
        return 1

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    fig.canvas.manager.set_window_title(f"ESP32 Sensor Viewer - {port}")
    series = [deque(maxlen=max(100, int(args.seconds * 50))) for _ in range(9)]
    times = deque(maxlen=series[0].maxlen)
    plots = []
    groups = ((1, 2), (3, 4, 5), (6, 7, 8))
    labels = (("Left", "Right"), ("Ax", "Ay", "Az"), ("Gx", "Gy", "Gz"))
    for axis, indices, names, ylabel in zip(axes, groups, labels,
                                            ("Angle (deg)", "Acceleration (g)", "Gyro (deg/s)")):
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend([axis.plot([], [], label=name)[0] for name in names], names, loc="upper right")
        plots.extend((axis.lines[-len(indices) + i], index) for i, index in enumerate(indices))
    axes[-1].set_xlabel("Time (s)")
    status = fig.text(0.02, 0.01, "Waiting for sensor_viewer data...", fontsize=9)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    pending = bytearray()
    origin = None
    last_time = None

    def update(_):
        nonlocal origin, last_time
        try:
            pending.extend(connection.read(min(connection.in_waiting, 8192)))
        except (serial.SerialException, OSError) as exc:
            status.set_text(f"Serial disconnected: {exc}")
            return []
        while b"\n" in pending:
            raw, _, rest = pending.partition(b"\n")
            pending[:] = rest
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("#"):
                print(line, flush=True)
                continue
            sample = parse_sample(line)
            if sample is None:
                continue
            now = sample[0] / 1000.0
            if origin is None or (last_time is not None and now < last_time):
                origin = now
                times.clear()
                for column in series:
                    column.clear()
            last_time = now
            times.append(now - origin)
            for i in range(1, 9):
                series[i].append(sample[i])
            status.set_text(f"{port} | left: {'OK' if sample[9] else 'ERROR'} | "
                            f"right: {'OK' if sample[10] else 'ERROR'} | "
                            f"MPU6050: {'OK' if sample[11] else 'ERROR'}")
        if len(pending) > 16384:
            pending.clear()
        if times:
            for line, index in plots:
                line.set_data(times, series[index])
            for axis in axes:
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)
                axis.set_xlim(max(0, times[-1] - args.seconds), max(args.seconds, times[-1]))
        return []

    animation = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
    fig._sensor_animation = animation  # Keep the timer alive until the window closes.
    try:
        plt.show()
    finally:
        connection.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
