import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "auto_pid", Path(__file__).resolve().parents[1] / "tools" / "auto_pid.py"
)
auto_pid = importlib.util.module_from_spec(spec)
spec.loader.exec_module(auto_pid)


def sample(ms, **changes):
    values = dict(ms=ms, pitch=0, set=0, lean=0, rate=0, wl=0, wr=0,
                  u=0, A=0, T=0, F=0, kp=8.8, ki=0, kd=0.29, guard=1)
    values.update(changes)
    return values


class AutoPidTest(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.tuner = auto_pid.AutoPid(self.sent.append)

    def test_guard_acknowledgement_required_before_start(self):
        self.tuner.begin(sample(0, guard=0), 0.0)
        self.assertEqual(self.sent, ["H"])
        self.tuner.on_sample(sample(50, guard=0), 0.05)
        self.assertNotIn("A", self.sent)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150), 0.15)
        self.assertIn("A", self.sent)

    def test_connection_loss_stops_session_and_never_rearms(self):
        self.tuner.begin(sample(0, guard=0), 0.0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.assertTrue(self.tuner.active)
        self.tuner.tick(1.0)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-1], "X")
        count = self.sent.count("A")
        self.tuner.on_sample(sample(1100), 1.1)
        self.assertEqual(self.sent.count("A"), count)

    def test_tilt_limit_cancels_trial(self):
        self.tuner.begin(sample(0, guard=0), 0.0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.assertTrue(self.tuner.active)
        self.tuner.on_sample(sample(200, A=1, pitch=13), 0.2)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-1], "X")

    def test_stationary_trace_is_not_convergence(self):
        self.tuner.begin(sample(0, guard=0), 0.0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        for i in range(1, 320):
            now = 0.15 + i * 0.05
            self.tuner.on_sample(sample(150+i*50, A=1), now)
            self.tuner.tick(now)
        self.assertFalse(self.tuner.converged)
        self.assertNotIn("converged", self.tuner.message.lower())

    def test_manual_stop_cancels_automatic_restarts(self):
        self.tuner.begin(sample(0, guard=0), 0.0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.cancel("operator stop")
        count = self.sent.count("A")
        self.tuner.on_sample(sample(150), 0.15)
        self.assertEqual(self.sent.count("A"), count)
        self.assertEqual(self.sent[-1], "X")

    def test_two_excited_supported_trials_can_meet_criteria(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        t = 150
        for trial in range(2):
            t += 50
            self.tuner.on_sample(sample(t, A=1, pitch=3), t / 1000)
            for i in range(1, 62):
                t += 50
                s = sample(t, A=1, pitch=0.5, wl=0.3 if i == 1 else 0,
                           wr=0.3 if i == 1 else 0, u=0.3 if i == 1 else 0)
                self.tuner.on_sample(s, t / 1000)
                self.tuner.tick(t / 1000)
            if trial == 0:
                self.assertTrue(self.tuner.active)
                t += 50
                self.tuner.on_sample(sample(t), t / 1000)
                t += 50
                self.tuner.on_sample(sample(t), t / 1000)
                t += 50
                self.tuner.on_sample(sample(t, A=1), t / 1000)
        t += 50
        self.tuner.on_sample(sample(t), t / 1000)
        self.assertTrue(self.tuner.converged)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-2:], ["X", "Q"])

    def test_no_wheel_response_aborts_instead_of_tuning(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.tuner.on_sample(sample(200, A=1, pitch=3), 0.2)
        for i in range(1, 62):
            ms = 200 + i * 50
            self.tuner.on_sample(sample(ms, A=1, pitch=0.5), ms / 1000)
            self.tuner.tick(ms / 1000)
        self.assertFalse(self.tuner.active)
        self.assertFalse(self.tuner.converged)
        self.assertIn("no measurable response", self.tuner.message)
        self.assertEqual(self.sent.count("A"), 1)

    def test_unsettled_trial_changes_one_gain_only_after_stop_ack(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.tuner.on_sample(sample(200, A=1, pitch=3), 0.2)
        for i in range(1, 61):
            ms = 200 + i * 50
            self.tuner.on_sample(sample(ms, A=1, pitch=2, wl=0.3, wr=0.3, u=0.3), ms / 1000)
            self.tuner.tick(ms / 1000)
        self.assertEqual(self.sent[-1], "X")
        self.assertNotIn("1", self.sent)
        self.tuner.on_sample(sample(3250), 3.25)
        self.assertEqual(self.sent[-1], "1")
        self.tuner.on_sample(sample(3300, kp=9.3), 3.3)
        self.tuner.on_sample(sample(3350, kp=9.3), 3.35)
        self.assertEqual(self.sent[-1], "A")

    def test_excessive_disturbance_cancels(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.tuner.on_sample(sample(200, A=1, pitch=6), 0.2)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-1], "X")

    def test_old_firmware_is_not_allowed_to_arm(self):
        old = sample(0)
        del old["guard"]
        self.tuner.begin(old, 0)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent, [])

    def test_missing_guard_after_start_aborts(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.tuner.on_sample(sample(200, A=1, guard=0), 0.2)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-1], "X")

    def test_sustained_saturation_stops_trial(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        for i in range(1, 20):
            ms = 150 + 50 * i
            self.tuner.on_sample(sample(ms, A=1, pitch=3, u=3, wl=0.3, wr=0.3), ms / 1000)
        self.assertFalse(self.tuner.active)
        self.assertIn("saturation", self.tuner.message)
        self.assertEqual(self.sent[-1], "X")

    def test_unexpected_gain_change_stops_trial(self):
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        self.tuner.on_sample(sample(200, A=1, kp=9.3), 0.2)
        self.assertFalse(self.tuner.active)
        self.assertEqual(self.sent[-1], "X")

    def test_trial_limit_restores_original_gains_while_stopped(self):
        self.tuner.max_trials = 2
        self.tuner.begin(sample(0, guard=0), 0)
        self.tuner.on_sample(sample(50), 0.05)
        self.tuner.on_sample(sample(100), 0.1)
        self.tuner.on_sample(sample(150, A=1), 0.15)
        t = 150
        for kp in (8.8, 9.3):
            t += 50
            self.tuner.on_sample(sample(t, A=1, pitch=3, kp=kp), t/1000)
            for _ in range(60):
                t += 50
                self.tuner.on_sample(sample(t, A=1, pitch=2, wl=.3, wr=.3, u=.3, kp=kp), t/1000)
                self.tuner.tick(t/1000)
            self.assertEqual(self.sent[-1], "X")
            t += 50
            self.tuner.on_sample(sample(t, kp=kp), t/1000)
            if kp == 8.8:
                self.assertEqual(self.sent[-1], "1")
                t += 50
                self.tuner.on_sample(sample(t, kp=9.3), t/1000)
                t += 50
                self.tuner.on_sample(sample(t, kp=9.3), t/1000)
                t += 50
                self.tuner.on_sample(sample(t, kp=9.3, A=1), t/1000)
            else:
                self.assertEqual(self.sent[-1], "0")
                t += 50
                self.tuner.on_sample(sample(t, kp=8.8), t/1000)
        self.assertFalse(self.tuner.active)
        self.assertFalse(self.tuner.converged)
        self.assertEqual(self.sent[-1], "Q")
        self.assertIn("initial gains restored", self.tuner.message)


if __name__ == "__main__":
    unittest.main()
