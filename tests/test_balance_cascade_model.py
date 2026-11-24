import unittest
from analysis.balance_model_identification.cascade_model import SpeedController


class SpeedControllerTests(unittest.TestCase):
    def test_forward_speed_request_demands_forward_lean(self):
        controller = SpeedController(kp=0.03, ki=0.01, limit=0.05)
        self.assertGreater(controller.update(reference=0.1, measured=0, dt=0.005), 0)

    def test_integral_does_not_wind_up_while_output_saturated(self):
        controller = SpeedController(kp=1, ki=1, limit=0.05)
        for _ in range(100):
            self.assertAlmostEqual(controller.update(1, 0, 0.005), 0.05)
        self.assertEqual(controller.integral, 0)


if __name__ == '__main__':
    unittest.main()
