"""Deterministic wheel-hold baseline; no claim of calibrated uncertainty."""

import heapq
import math
from collections import Counter, OrderedDict
from statistics import median

from .types import ControlSample, Estimate, EstimatorConfig, WheelSample, event_key


class OdometryEstimator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.config = EstimatorConfig()
        self.t = None
        self.s = self.v = 0.0
        self.control = None
        self.wheels = {}
        self.queue = []
        self.seen = OrderedDict()
        self.counts = Counter()
        self.pending_reasons = set()
        self.output_seq = 0
        self.serial = 0
        self.closed_time = None

    def initialize(self, initial, config):
        if (
            initial.stamp_ns < 0
            or not all(math.isfinite(x) for x in (initial.s_m, initial.v_mps))
            or initial.v_mps < 0
        ):
            raise ValueError("Invalid forward-motion initial state")
        limits = (config.wheel_timeout_s, config.control_timeout_s, config.max_hold_s)
        if (
            not all(math.isfinite(x) and x > 0 for x in limits)
            or config.max_hold_s < config.wheel_timeout_s
            or min(config.max_channels, config.max_queue, config.dedup_capacity) < 1
        ):
            raise ValueError("Invalid estimator limits")
        self.reset()
        self.config = config
        self.t, self.s, self.v = initial.stamp_ns, initial.s_m, initial.v_mps

    def _reject(self, code, wheel=False):
        self.counts[code] += 1
        self.pending_reasons.add(code)
        if wheel:
            self.counts["rejected_wheels"] += 1

    def ingest_control(self, sample):
        self._ingest(sample)

    def ingest_wheel(self, sample):
        self._ingest(sample)

    def _ingest(self, event):
        wheel = isinstance(event, WheelSample)
        value = event.speed_mps if wheel else event.u
        if (
            not isinstance(event.stamp_ns, int)
            or not isinstance(event.seq, int)
            or event.stamp_ns < 0
            or event.seq < 0
            or not math.isfinite(value)
            or (wheel and (not event.wheel_id or value < 0))
            or (not wheel and not -1 <= value <= 1)
        ):
            self._reject("INVALID_INPUT", wheel)
            return
        key = ("wheel", event.wheel_id, event.seq) if wheel else ("control", event.seq)
        if key in self.seen:
            self._reject("DUPLICATE", wheel)
            return
        if (self.t is not None and event.stamp_ns < self.t) or (
            self.closed_time is not None and event.stamp_ns <= self.closed_time
        ):
            self._reject("OUT_OF_ORDER", wheel)
            return
        if len(self.queue) >= self.config.max_queue:
            self._reject("QUEUE_OVERFLOW", wheel)
            return
        self.seen[key] = None
        if len(self.seen) > self.config.dedup_capacity:
            self.seen.popitem(last=False)
        self.serial += 1
        heapq.heappush(self.queue, (event_key(event), self.serial, event))

    def _fresh(self, timestamp):
        timeout = round(self.config.wheel_timeout_s * 1e9)
        return {
            key: sample.speed_mps
            for key, sample in self.wheels.items()
            if sample.valid and timestamp - sample.stamp_ns < timeout
        }

    def _move(self, target):
        # Recompute the median at expiry boundaries, not just at publication ticks.
        timeout = round(self.config.wheel_timeout_s * 1e9)
        boundaries = sorted(
            {
                w.stamp_ns + timeout
                for w in self.wheels.values()
                if w.valid and self.t < w.stamp_ns + timeout <= target
            }
        )
        for endpoint in boundaries + [target]:
            fresh = self._fresh(self.t)
            if fresh:
                self.v = median(fresh.values())
            self.s += self.v * ((endpoint - self.t) / 1e9)
            self.t = endpoint

    def advance_to(self, stamp_ns):
        if stamp_ns < 0 or (self.t is not None and stamp_ns < self.t):
            raise ValueError("Clock moved backwards: reset and initialize a new run")
        if self.t is None:
            return Estimate(
                stamp_ns, self.output_seq, None, None, "INITIALIZING", False, ["INITIAL_STATE_REQUIRED"]
            )
        while self.queue and self.queue[0][0][0] <= stamp_ns:
            _, _, event = heapq.heappop(self.queue)
            self._move(event.stamp_ns)
            if isinstance(event, ControlSample):
                self.control = event
            elif event.wheel_id not in self.wheels and len(self.wheels) >= self.config.max_channels:
                self._reject("CHANNEL_LIMIT", True)
            else:
                self.wheels[event.wheel_id] = event
                self.counts["accepted_wheels" if event.valid else "rejected_wheels"] += 1
            fresh = self._fresh(self.t)
            if fresh:
                self.v = median(fresh.values())
        self._move(stamp_ns)
        self.closed_time = stamp_ns
        self.output_seq += 1
        fresh = self._fresh(stamp_ns)
        if fresh:
            self.v = median(fresh.values())
        valid_wheels = [w for w in self.wheels.values() if w.valid]
        wheel_age = (stamp_ns - max(w.stamp_ns for w in valid_wheels)) / 1e9 if valid_wheels else None
        control_age = (stamp_ns - self.control.stamp_ns) / 1e9 if self.control else None
        reasons = sorted(self.pending_reasons) + ["UNCERTAINTY_UNCALIBRATED", "BASELINE_ONLY"]
        self.pending_reasons.clear()
        control_ok = (
            self.control is not None and self.control.valid and control_age < self.config.control_timeout_s
        )
        if not control_ok:
            reasons.append("CONTROL_STALE")
        if not fresh:
            reasons.append("WHEEL_STALE")
        valid = control_ok and wheel_age is not None and wheel_age < self.config.max_hold_s
        mode = ("FUSED" if fresh else "DEGRADED") if valid else "INVALID"
        if fresh and len(fresh) < len(self.wheels) and valid:
            mode = "DEGRADED"
        if not math.isfinite(self.s) or not math.isfinite(self.v):
            self.s = self.v = 0.0
            return Estimate(
                stamp_ns, self.output_seq, None, None, "INVALID", False, reasons + ["NUMERICAL_FAILURE"]
            )
        return Estimate(
            stamp_ns,
            self.output_seq,
            self.s,
            self.v,
            mode,
            valid,
            reasons,
            wheel_speeds_mps=fresh,
            control_u=self.control.u if self.control else None,
            control_age_s=control_age,
            wheel_age_s=wheel_age,
            accepted_wheel_count=self.counts["accepted_wheels"],
            rejected_wheel_count=self.counts["rejected_wheels"],
        )
