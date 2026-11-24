import unittest
import numpy as np
from analysis.balance_model_identification.model import Parameters, derivative, linearize


class BalanceModelTests(unittest.TestCase):
    def test_upright_is_an_equilibrium_but_small_lean_falls(self):
        p = Parameters()
        np.testing.assert_allclose(derivative(np.zeros(4), 0, p), 0, atol=1e-12)
        self.assertGreater(derivative(np.array([0, 0, 0.01, 0]), 0, p)[3], 0)

    def test_linearization_matches_nonlinear_small_signal(self):
        p = Parameters()
        a, b = linearize(p)
        state = np.array([1e-7, -2e-7, 3e-7, -4e-7])
        u = 2e-7
        np.testing.assert_allclose(derivative(state, u, p), a @ state + b * u,
                                   rtol=1e-9, atol=1e-12)

    def test_positive_voltage_produces_forward_acceleration_and_body_reaction(self):
        result = derivative(np.zeros(4), 1, Parameters())
        self.assertGreater(result[1], 0)
        self.assertLess(result[3], 0)


if __name__ == '__main__':
    unittest.main()
