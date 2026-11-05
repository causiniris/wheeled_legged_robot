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

            def close(self):
                self.closed = True

        connection = FakeSerial()

        def show():
            fig = plt.gcf()
            fig._balance_animation._func(0)
            self.assertEqual(list(fig.axes[0].lines[0].get_ydata()), [2.0])
            self.assertEqual(list(fig.axes[2].lines[0].get_ydata()), [0.5])
            self.assertIn("Kp=8.80 Ki=0.00 Kd=0.29", fig.texts[0].get_text())
            fig._balance_animation._draw_was_started = True

        with patch("sys.argv", ["balance_viewer.py", "--port", "COM10"]), \
             patch("serial.Serial", return_value=connection), patch("matplotlib.pyplot.show", side_effect=show):
            self.assertEqual(viewer.main(), 0)
        self.assertEqual(connection.sent[-1], b"X")
        self.assertTrue(connection.closed)
        plt.close("all")


if __name__ == "__main__":
    unittest.main()
