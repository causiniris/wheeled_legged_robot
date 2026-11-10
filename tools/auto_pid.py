"""Bounded, supervised balance-controller trials; never a stability certificate."""
import math


def angle_error(sample):
    return (sample["pitch"] - sample["set"] - sample["lean"] + 180) % 360 - 180


class AutoPid:
    # Commands are firmware-defined increments. Never search beyond these ranges.
    axes = (("kp", 0.5, "1", "0", 2.0), ("kd", 0.02, "3", "2", 0.08),
            ("ki", 0.1, "5", "4", 0.2))
    max_trials = 10

    def __init__(self, send):
        self.send = send
        self.active = False
        self.converged = False
        self.message = "Auto PID idle"
        self.state = "idle"
        self.trials = 0

    def _send(self, command):
        try:
            self.send(command)
            return True
        except (OSError, IOError):
            self.active = False
            self.message = "Serial write failed; cut motor power."
            return False

    def begin(self, sample, now):
        if self.active:
            return
        self.converged = False
        if (sample is None or sample.get("guard") != 0 or sample["A"] or sample["T"]
                or sample["F"] or not (2 <= sample["kp"] <= 20)
                or not (0 <= sample["ki"] <= 0.5) or not (0 <= sample["kd"] <= 1)):
            self.message = "Cannot tune: need current guarded-firmware telemetry, stopped and fault-free, with moderate gains."
            return
        self.base = {key: sample[key] for key in ("kp", "ki", "kd")}
        self.best = self.base.copy()
        self.best_score = float("inf")
        self.target = self.base.copy()
        self.axis_index = 0
        self.sign = 1
        self.trials = self.confirmations = 0
        self.pending_convergence = False
        self.restoring = False
        self.saturation_since = None
        self.active = True
        self.state = "guard"
        self.last_heartbeat = self.last_wall = now
        self.last_ms = sample["ms"]
        self.wait_since = now
        self.message = "Enabling device heartbeat guard"
        self._send("H")

    def cancel(self, reason="operator stop"):
        if not self.active:
            return
        self.active = False
        self.state = "stopped"
        self.message = "Auto PID stopped: " + reason
        self._send("X")

    def tick(self, now):
        if not self.active:
            return
        if now - self.last_wall > 0.7:
            self.cancel("telemetry timeout; check power and serial")
            return
        timeout = 15 if self.state in ("ready", "excite") else 5 if self.state == "observe" else 3
        if now - self.wait_since > timeout:
            self.cancel("timeout in " + self.state)
            return
        if now - self.last_heartbeat >= 0.15:
            if self._send("H"):
                self.last_heartbeat = now

    def _set_state(self, state, now, message):
        self.state = state
        self.wait_since = now
        self.message = message

    def _next_candidate(self):
        if self.trials >= self.max_trials:
            return False
        key, step, _, _, max_delta = self.axes[self.axis_index]
        proposed = self.best.copy()
        proposed[key] = round(proposed[key] + self.sign * step, 2)
        self.sign *= -1
        if self.sign == 1:
            self.axis_index = (self.axis_index + 1) % len(self.axes)
        limits = {"kp": (2, 20), "ki": (0, 0.5), "kd": (0, 1)}
        low, high = limits[key]
        if not low <= proposed[key] <= high or abs(proposed[key] - self.base[key]) > max_delta + 1e-6:
            return self._next_candidate()
        self.target = proposed
        return True

    def _apply_next_gain(self, sample, now):
        for key, step, up, down, _ in self.axes:
            delta = self.target[key] - sample[key]
            if abs(delta) > step / 2:
                if self._send(up if delta > 0 else down):
                    self.expected = (key, round(sample[key] + (step if delta > 0 else -step), 2))
                    self._set_state("gain_ack", now, "Applying " + key)
                return
        if self.restoring:
            if self._send("Q"):
                self.active = False
                self.state = "finished"
                self.message = "Trial limit reached without convergence; initial gains restored."
            return
        self._set_state("ready", now, "Hold upright and still; preparing next trial")

    def _finish_trial(self, now):
        self.trials += 1
        tail = self.trace[-20:]
        score = sum(abs(e) + 0.05 * abs(rate) for e, rate, _, _, _ in self.trace) / len(self.trace)
        settled = (len(tail) >= 15 and max(abs(row[0]) for row in tail) <= 1.5
                   and max(abs(row[1]) for row in tail) <= 5
                   and max(max(abs(row[2]), abs(row[3])) for row in tail) <= 1)
        moved = (max(abs(row[4]) for row in self.trace) >= 0.15
                 and max(abs(row[2]) for row in self.trace) >= 0.2
                 and max(abs(row[3]) for row in self.trace) >= 0.2)
        if not moved:
            self.cancel("no measurable response from both wheels; check drive and load")
            return
        if score < self.best_score:
            self.best_score, self.best = score, self.target.copy()
        print(f"Auto PID trial {self.trials}: Kp={self.target['kp']:.2f} "
              f"Ki={self.target['ki']:.2f} Kd={self.target['kd']:.2f} "
              f"score={score:.3f} settled={int(settled)}", flush=True)
        self.confirmations = self.confirmations + 1 if settled else 0
        self._send("X")
        if not self.active:
            return
        if self.confirmations >= 2:
            self.pending_convergence = True
            self._set_state("finish_ack", now, "Waiting for motor stop")
            return
        if self.trials >= self.max_trials or (self.confirmations == 0 and not self._next_candidate()):
            self.target = self.base.copy()
            self.restoring = True
            self._set_state("stop_ack", now, "Trial limit reached; restoring initial gains")
            return
        self._set_state("stop_ack", now, f"Trial {self.trials} complete; waiting for stop")

    def on_sample(self, sample, now):
        if not self.active:
            return
        if sample["ms"] <= self.last_ms:
            self.cancel("device reset or out-of-order telemetry")
            return
        if sample["ms"] - self.last_ms > 400:
            self.cancel("telemetry gap")
            return
        self.last_ms, self.last_wall = sample["ms"], now
        if sample["F"] or sample["T"] or (self.state != "guard" and sample.get("guard") != 1):
            self.cancel("fault, wheel test, or heartbeat guard unavailable")
            return
        if self.state in ("ready", "arm_ack", "excite", "observe") and any(
                abs(sample[key] - self.target[key]) > 0.011 for key in ("kp", "ki", "kd")):
            self.cancel("unexpected gain change")
            return
        err = angle_error(sample)
        if (abs(err) > 12 or abs(sample["rate"]) > 50
                or abs(sample["wl"]) > 12 or abs(sample["wr"]) > 12):
            self.cancel("trial safety limit")
            return
        if sample["A"] and abs(sample["u"]) >= 2.95:
            if self.saturation_since is None:
                self.saturation_since = sample["ms"]
            elif sample["ms"] - self.saturation_since >= 600:
                self.cancel("sustained output saturation")
                return
        else:
            self.saturation_since = None
        if self.state == "guard":
            if sample.get("guard") == 1 and not sample["A"]:
                self._apply_next_gain(sample, now)
        elif self.state == "gain_ack":
            key, expected = self.expected
            if sample["A"]:
                self.cancel("motor unexpectedly enabled while setting gains")
            elif abs(sample[key] - expected) < 0.011:
                self._apply_next_gain(sample, now)
        elif self.state == "ready":
            if sample["A"]:
                self.cancel("motor unexpectedly enabled")
            elif abs(err) < 3 and abs(sample["rate"]) < 5 and abs(sample["wl"]) < 0.5 and abs(sample["wr"]) < 0.5:
                if self._send("A"):
                    self._set_state("arm_ack", now, "Starting trial")
        elif self.state == "arm_ack":
            if sample["A"]:
                self._set_state("excite", now, "Gently lean 2-5 degrees, then release support slightly")
        elif self.state == "excite":
            if not sample["A"]:
                self.cancel("motor stopped during trial")
            elif abs(err) > 5:
                self.cancel("disturbance exceeded 5 degrees")
            elif 2 <= abs(err) <= 5:
                self.trace = []
                self.trace_start = sample["ms"]
                self._set_state("observe", now, "Measuring response")
        elif self.state == "observe":
            if not sample["A"]:
                self.cancel("motor stopped during trial")
                return
            self.trace.append((err, sample["rate"], sample["wl"], sample["wr"], sample["u"]))
            if sample["ms"] - self.trace_start >= 3000:
                self._finish_trial(now)
        elif self.state == "stop_ack":
            if not sample["A"]:
                self._apply_next_gain(sample, now)
        elif self.state == "finish_ack":
            if not sample["A"] and self._send("Q"):
                self.active = False
                self.state = "finished"
                self.converged = self.pending_convergence
                if self.converged:
                    self.message = ("Two supported trials met thresholds; not proof of free balance. "
                                    f"Kp={self.target['kp']:.2f} Ki={self.target['ki']:.2f} Kd={self.target['kd']:.2f}")
                else:
                    self.message = "Trial limit reached without convergence; inspect measurements."
