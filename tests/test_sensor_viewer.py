import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "sensor_viewer", Path(__file__).resolve().parents[1] / "tools" / "sensor_viewer.py"
)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)


class ParseSampleTest(unittest.TestCase):
    def test_valid_sample_and_missing_sensor(self):
        sample = viewer.parse_sample("1500,90,180,nan,nan,nan,nan,nan,nan,1,1,0")
        self.assertEqual(sample[0:3], [1500, 90, 180])
        self.assertTrue(math.isnan(sample[3]))
        self.assertEqual(sample[-3:], [1, 1, 0])

    def test_ignore_boot_header_and_invalid_rows(self):
        for line in ("# LEFT scan: 0x06", "ms,left_deg", "100,1,2", "nan,1,2,3,4,5,6,7,8,1,1,1",
                     "100,1,2,3,4,5,6,7,8,1,1,2"):
            self.assertIsNone(viewer.parse_sample(line))

    def test_window_plots_data_and_closes_serial(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        class FakeSerial:
            def __init__(self, *args, **kwargs):
                self.data = b"1000,90,180,0,0,1,1,2,3,1,1,1\n"
                self.closed = False

            @property
            def in_waiting(self):
                return len(self.data)

            def read(self, size):
                result, self.data = self.data[:size], self.data[size:]
                return result

            def close(self):
                self.closed = True

        connection = FakeSerial()

        def show():
            fig = plt.gcf()
            fig._sensor_animation._func(0)
            self.assertEqual(list(fig.axes[0].lines[0].get_ydata()), [90.0])
            self.assertEqual(list(fig.axes[1].lines[2].get_ydata()), [1.0])
            self.assertEqual(list(fig.axes[2].lines[2].get_ydata()), [3.0])
            self.assertIn("MPU6050: OK", fig.texts[0].get_text())
            fig._sensor_animation._draw_was_started = True

        with patch("sys.argv", ["sensor_viewer.py", "--port", "COM3"]), \
             patch("serial.Serial", return_value=connection), patch("matplotlib.pyplot.show", side_effect=show):
            self.assertEqual(viewer.main(), 0)
        self.assertTrue(connection.closed)
        plt.close("all")


if __name__ == "__main__":
    unittest.main()
