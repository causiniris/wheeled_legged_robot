import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "balance_viewer", Path(__file__).resolve().parents[1] / "tools" / "balance_viewer.py"
)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)

ROW = "ms=1000 pitch=2 set=0 rate=3 wl=1 wr=2 u=0.5 lean=0 A=1 T=0 F=0 kp=8.8 ki=0 kd=0.29"


class BalanceViewerTest(unittest.TestCase):
    def test_parse_telemetry(self):
        sample = viewer.parse_sample(ROW)
        self.assertEqual(sample["pitch"], 2)
        self.assertEqual(sample["ki"], 0)
        self.assertIsNone(viewer.parse_sample("FAULT: invalid"))
        self.assertIsNone(viewer.parse_sample(ROW.replace("pitch=2", "pitch=nan")))
        self.assertIsNone(viewer.parse_sample(ROW.replace("A=1", "A=2")))
        self.assertNotIn("guard", sample)
        self.assertEqual(viewer.parse_sample(ROW + " guard=1")["guard"], 1)
        self.assertIsNone(viewer.parse_sample(ROW + " guard=2"))

    def test_window_plots_and_sends_stop_on_close(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        class FakeSerial:
            def __init__(self):
                self.data = (ROW + "\n").encode()
                self.sent = []
                self.closed = False

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, size):
                result, self.data = self.data[:size], self.data[size:]
                return result

            def write(self, data):
                self.sent.append(data)
                return len(data)

            def close(self):
                self.closed = True

        connection = FakeSerial()

        def show():
            fig = plt.gcf()
            with patch("time.monotonic", return_value=100.0):
                fig._balance_animation._func(0)
            self.assertEqual(list(fig.axes[0].lines[0].get_ydata()), [2.0])
            self.assertEqual(list(fig.axes[2].lines[0].get_ydata()), [0.5])
            self.assertIn("Kp=8.80 Ki=0.00 Kd=0.29", fig.texts[0].get_text())
            connection.data = (b"FAULT: motor alignment; check phases, magnets, supply and pole pairs.\n"
                               b"MT6701 RIGHT SDA17/SCL16 addr=0x06: I2C_SHORT_READ reg=0x03 received=0\n"
                               b"Cannot start: initialization/sensor fault; check and reset.\n")
            with patch("time.monotonic", return_value=102.0):
                fig._balance_animation._func(0)
            self.assertIn("RIGHT SDA17/SCL16 addr=0x06: I2C_SHORT_READ", fig.texts[0].get_text())
            connection.data = (ROW + "\n").encode()
            with patch("time.monotonic", return_value=103.0):
                fig._balance_animation._func(0)
            self.assertIn("Kp=8.80", fig.texts[0].get_text())
            fig._balance_buttons[4]._observers.process("clicked", None)
            self.assertEqual(connection.sent[-1], b"R")
            idle = ROW.replace("A=1", "A=0")
            reply = "Cannot start: hold upright, stop wheels, and wait for fresh IMU readings."
            connection.data = (reply + "\n" + idle + "\n").encode()
            fig._balance_animation._func(0)
            self.assertEqual(fig.texts[1].get_text(), reply)
            connection.data = ("Wheel test: RIGHT 0.5 V for 200 ms.\n" + idle.replace("T=0", "T=2") + "\n").encode()
            fig._balance_animation._func(0)
            self.assertIn("T=2", fig.texts[0].get_text())
            self.assertIn("RIGHT 0.5 V", fig.texts[1].get_text())
            overspeed = "OVERSPEED: A=1 T=0 on_ms=5 wl=31.00 wr=7.00 ul_cmd=1.25 ur_cmd=1.25"
            connection.data = ("STOP: wheel overspeed\n" + overspeed + "\n" + idle + "\n").encode()
            fig._balance_animation._func(0)
            self.assertIn("A=0 T=0 F=0", fig.texts[0].get_text())
            self.assertEqual(fig.texts[1].get_text(), overspeed)
            waiting = "IMU WAIT: accelerometer saturated while disarmed. Hold upright and still."
            connection.data = (waiting + "\n").encode()
            fig._balance_animation._func(0)
            self.assertEqual(fig.texts[1].get_text(), waiting)
            recovered = "IMU RECOVERED: sample ready. Motors disarmed."
            connection.data = (recovered + "\n" + idle + "\n").encode()
            fig._balance_animation._func(0)
            self.assertEqual(fig.texts[1].get_text(), recovered)
            fig._balance_animation._draw_was_started = True

        with patch("sys.argv", ["balance_viewer.py", "--port", "COM10"]), \
             patch("serial.Serial", return_value=connection), patch("matplotlib.pyplot.show", side_effect=show):
            self.assertEqual(viewer.main(), 0)
        self.assertEqual(connection.sent[-1], b"X")
        self.assertTrue(connection.closed)
        plt.close("all")

    def test_auto_button_requires_guard_and_stop_cancels(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        class FakeSerial:
            def __init__(self):
                self.data = b""
                self.sent = []
            @property
            def in_waiting(self):
                return len(self.data)
            def read(self, size):
                result, self.data = self.data[:size], self.data[size:]
                return result
            def write(self, data):
                self.sent.append(data)
                return len(data)
            def close(self):
                pass
            def frame(self, ms, guard):
                self.data = (ROW.replace("ms=1000", f"ms={ms}").replace("pitch=2", "pitch=0")
                             .replace("set=0", "set=0").replace("rate=3", "rate=0")
                             .replace("wl=1", "wl=0").replace("wr=2", "wr=0")
                             .replace("A=1", "A=0") + f" guard={guard}\n").encode()

        connection = FakeSerial()
        def show():
            fig = plt.gcf()
            connection.frame(1000, 0)
            fig._balance_animation._func(0)
            fig._balance_buttons[-1]._observers.process("clicked", None)
            self.assertEqual(connection.sent, [b"H"])
            connection.frame(1050, 1)
            fig._balance_animation._func(0)
            connection.frame(1100, 1)
            fig._balance_animation._func(0)
            self.assertIn(b"A", connection.sent)
            fig._balance_buttons[1]._observers.process("clicked", None)
            self.assertEqual(connection.sent[-1], b"X")
            fig._balance_animation._draw_was_started = True

        with patch("sys.argv", ["balance_viewer.py", "--port", "COM7"]), \
             patch("serial.Serial", return_value=connection), patch("matplotlib.pyplot.show", side_effect=show):
            self.assertEqual(viewer.main(), 0)
        plt.close("all")


if __name__ == "__main__":
    unittest.main()
