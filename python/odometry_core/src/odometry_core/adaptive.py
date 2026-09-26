"""Four-state longitudinal EKF with robust multi-wheel fault handling.

The state is ``[position, velocity, actuator acceleration, disturbance]``.
Parameters are intentionally explicit: they are starting values that must be
calibrated on the organizer's data, not claims about a particular tram.
"""

from __future__ import annotations

import heapq
import math
from collections import OrderedDict
from dataclasses import dataclass, fields
from statistics import median

from .types import ControlSample, Estimate, InitialState, WheelSample, event_key

N = 4


@dataclass(frozen=True)
class ModelConfig:
    kt: float = 1.2
    kb: float = 1.5
    u_dead: float = 0.05
    v_sat: float = 40.0
    v_brake: float = 0.5
    tau_s: float = 0.3
    c1: float = 0.01
    c2: float = 0.0
    q_v: float = 0.01
    q_a: float = 0.05
    q_d: float = 0.001
    p0_s: float = 1.0
    p0_v: float = 0.25
    p0_a: float = 1.0
    p0_d: float = 0.25
    wheel_variance_floor: float = 0.04
    gate_normal: float = 9.0
    gate_reject: float = 36.0
    max_wheel_accel: float = 6.0
    gate_sigma_cap: float = 1.0
    reacquire_s: float = 10.0
    recover_updates: int = 5
    detect_freeze: bool = True
    channel_gate: bool = True
    freeze_s: float = 1.0
    freeze_min_accel: float = 0.2
    reason_hold_s: float = 1.0
    wheel_timeout_s: float = 0.5
    control_timeout_s: float | None = None
    max_model_only_s: float = 10.0
    max_step_s: float = 0.02
    max_gap_s: float = 60.0
    max_channels: int = 64
    max_queue: int = 4096
    dedup_capacity: int = 8192
    use_wheels: bool = True
    robust: bool = True
    adapt_disturbance: bool = False

    def __post_init__(self):
        positive = (
            "kt",
            "kb",
            "v_sat",
            "v_brake",
            "tau_s",
            "wheel_variance_floor",
            "gate_normal",
            "gate_reject",
            "max_wheel_accel",
            "gate_sigma_cap",
            "reacquire_s",
            "freeze_s",
            "freeze_min_accel",
            "wheel_timeout_s",
            "max_model_only_s",
            "max_step_s",
            "max_gap_s",
            "p0_s",
            "p0_v",
            "p0_a",
            "p0_d",
        )
        nonnegative = ("u_dead", "c1", "c2", "q_v", "q_a", "q_d", "reason_hold_s")
        if any(not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0 for name in positive):
            raise ValueError("Model parameters marked positive must be finite and positive")
        if any(not math.isfinite(getattr(self, name)) or getattr(self, name) < 0 for name in nonnegative):
            raise ValueError("Noise, drag and dead-zone parameters must be finite and non-negative")
        if self.gate_reject < self.gate_normal or self.recover_updates < 1:
            raise ValueError("gate_reject must be >= gate_normal and recover_updates >= 1")
        if self.control_timeout_s is not None and self.control_timeout_s <= 0:
            raise ValueError("control_timeout_s must be positive or null")
        if min(self.max_channels, self.max_queue, self.dedup_capacity) < 1:
            raise ValueError("Queue and channel limits must be positive")

    @classmethod
    def from_dict(cls, values: dict) -> "ModelConfig":
        known = {item.name for item in fields(cls)}
        unknown = set(values) - known
        if unknown:
            raise ValueError(f"Unknown model parameters: {sorted(unknown)}")
        return cls(**values)


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(N)) for j in range(N)] for i in range(N)]


def _transpose(a):
    return [[a[j][i] for j in range(N)] for i in range(N)]


class AdaptiveOdometryEstimator:
    """Transport-independent robust EKF implementing the common estimator API."""

    MODEL_VERSION = "ekf-robust-v1"

    def __init__(self, model_config: ModelConfig | None = None):
        self.default_config = model_config or ModelConfig()
        self.reset()

    def reset(self):
        self.config = self.default_config
        self.t = None
        self.closed_time = None
        self.x = [0.0] * N
        self.P = [[0.0] * N for _ in range(N)]
        self.initialized = False
        self.control = None
        self.control_valid = False
        self.last_control = None
        self.channels: dict[str, WheelSample] = {}
        self.queue = []
        self.seen = OrderedDict()
        self.serial = 0
        self.output_seq = 0
        self.accepted = self.rejected = 0
        self.quality = "FUSED"
        self.streak = self.consistent = 0
        self.last_accept = self.last_reject = self.last_downweight = None
        self.last_reacquire = self.last_frozen = self.last_reset = None
        self.z_prev = self.z_time = None
        self.frozen_ids: set[str] = set()
        self.hold = {}
        self.pending_reasons: set[str] = set()
        self.clamped = False

    def initialize(self, initial: InitialState, config=None):
        self.reset()
        self.config = self.default_config if config is None or not isinstance(config, ModelConfig) else config
        if (
            initial.stamp_ns < 0
            or initial.v_mps < 0
            or not all(math.isfinite(value) for value in (initial.s_m, initial.v_mps))
        ):
            raise ValueError("Invalid forward-motion initial state")
        self.t = initial.stamp_ns
        self.x = [initial.s_m, initial.v_mps, 0.0, 0.0]
        covariance = initial.covariance_4x4
        if covariance is not None:
            flat = list(covariance)
            if len(flat) != 16 or not all(math.isfinite(value) for value in flat):
                raise ValueError("covariance_4x4 must contain 16 finite row-major values")
            self.P = [flat[index : index + N] for index in range(0, 16, N)]
        else:
            diag = (
                self.config.p0_s,
                self.config.p0_v,
                self.config.p0_a,
                self.config.p0_d if self.config.adapt_disturbance else 1e-12,
            )
            self.P = [[0.0] * N for _ in range(N)]
            for index, value in enumerate(diag):
                self.P[index][index] = value
        self.initialized = True
        self.last_accept = initial.stamp_ns

    def ingest_control(self, sample: ControlSample):
        self._enqueue(sample)

    def ingest_wheel(self, sample: WheelSample):
        self._enqueue(sample)

    def _enqueue(self, event):
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
            self.pending_reasons.add("INVALID_INPUT")
            self.rejected += int(wheel)
            return
        key = ("wheel", event.wheel_id, event.seq) if wheel else ("control", event.seq)
        if key in self.seen:
            self.pending_reasons.add("DUPLICATE")
            self.rejected += int(wheel)
            return
        if event.stamp_ns < self.t or (self.closed_time is not None and event.stamp_ns <= self.closed_time):
            self.pending_reasons.add("OUT_OF_ORDER")
            self.rejected += int(wheel)
            return
        if len(self.queue) >= self.config.max_queue:
            self.pending_reasons.add("QUEUE_OVERFLOW")
            self.rejected += int(wheel)
            return
        self.seen[key] = None
        if len(self.seen) > self.config.dedup_capacity:
            self.seen.popitem(last=False)
        self.serial += 1
        heapq.heappush(self.queue, (event_key(event), self.serial, event))

    def _command(self, u, v):
        c = self.config
        drive, brake = max(u - c.u_dead, 0.0), max(-u - c.u_dead, 0.0)
        gain = 1.0 / (1.0 + v / c.v_sat)
        braking = min(1.0, v / c.v_brake) if v > 0 else 0.0
        derivative = 1.0 / c.v_brake if 0 < v < c.v_brake else 0.0
        return (
            c.kt * drive * gain - c.kb * brake * braking,
            -c.kt * drive * gain * gain / c.v_sat - c.kb * brake * derivative,
        )

    def _step(self, dt):
        c = self.config
        s, v, acceleration, disturbance = self.x
        decay = math.exp(-dt / c.tau_s)
        alpha = 1.0 - decay
        command, derivative = self._command(self.control or 0.0, v)
        drag_derivative = c.c1 + 2 * c.c2 * abs(v)
        state = [
            s + v * dt,
            v + (acceleration - c.c1 * v - c.c2 * v * abs(v) + disturbance) * dt,
            acceleration + (command - acceleration) * alpha,
            disturbance,
        ]
        transition = [
            [1, dt, 0, 0],
            [0, 1 - drag_derivative * dt, dt, dt],
            [0, alpha * derivative, decay, 0],
            [0, 0, 0, 1],
        ]
        covariance = _mm(_mm(transition, self.P), _transpose(transition))
        for index, noise in ((1, c.q_v), (2, c.q_a), (3, c.q_d if c.adapt_disturbance else 0.0)):
            covariance[index][index] += noise * dt
        if state[1] < 0:
            state[1] = 0.0
            self.clamped = True
        self.x, self.P = state, covariance

    def _predict_to(self, stamp):
        if stamp < self.t:
            raise ValueError("Estimator time cannot move backwards")
        total = (stamp - self.t) / 1e9
        if total > self.config.max_gap_s:
            position = self.x[0]
            initial = InitialState(stamp, position, self.x[1])
            self.initialize(initial, self.config)
            self.last_reset = stamp
            return
        if total:
            count = max(1, math.ceil(total / self.config.max_step_s))
            for _ in range(count):
                self._step(total / count)
        self.t = stamp

    def _update(self, measurement, variance, full=True):
        innovation_variance = self.P[1][1] + variance
        gain = [self.P[index][1] / innovation_variance for index in range(N)]
        if not full:
            gain[2] = gain[3] = 0.0
        if not self.config.adapt_disturbance:
            gain[3] = 0.0
        error = measurement - self.x[1]
        self.x = [self.x[index] + gain[index] * error for index in range(N)]
        ikh = [[(1.0 if i == j else 0.0) - (gain[i] if j == 1 else 0.0) for j in range(N)] for i in range(N)]
        covariance = _mm(_mm(ikh, self.P), _transpose(ikh))
        for i in range(N):
            for j in range(N):
                covariance[i][j] += gain[i] * variance * gain[j]
        for i in range(N):
            for j in range(i):
                covariance[i][j] = covariance[j][i] = (covariance[i][j] + covariance[j][i]) / 2
            covariance[i][i] = max(covariance[i][i], 1e-12)
        self.P = covariance
        self.x[1] = max(self.x[1], 0.0)

    def _is_frozen(self, sample):
        previous = self.hold.get(sample.wheel_id)
        if previous is None or previous[0] != sample.speed_mps:
            self.hold[sample.wheel_id] = (sample.speed_mps, sample.stamp_ns)
            return False
        elapsed = (sample.stamp_ns - previous[1]) / 1e9
        expected = self._command(self.control or 0.0, self.x[1])[0] - self.config.c1 * self.x[1]
        return (
            self.config.detect_freeze
            and elapsed >= self.config.freeze_s
            and abs(expected) >= self.config.freeze_min_accel
        )

    def _process_group(self, stamp, events):
        self._predict_to(stamp)
        candidates = []
        for event in events:
            if isinstance(event, ControlSample):
                self.control = event.u if event.valid else None
                self.control_valid = event.valid
                self.last_control = stamp
                continue
            if event.wheel_id not in self.channels and len(self.channels) >= self.config.max_channels:
                self.pending_reasons.add("CHANNEL_LIMIT")
                self.rejected += 1
                continue
            self.channels[event.wheel_id] = event
            if not event.valid:
                self.rejected += 1
            elif self._is_frozen(event):
                self.frozen_ids.add(event.wheel_id)
                self.last_frozen = stamp
                self.rejected += 1
            else:
                self.frozen_ids.discard(event.wheel_id)
                candidates.append(event)
        if not candidates or not self.config.use_wheels:
            return
        if self.config.robust and self.config.channel_gate and len(candidates) >= 2:
            variance = self.config.wheel_variance_floor + min(self.P[1][1], self.config.gate_sigma_cap**2)
            inliers = [
                item
                for item in candidates
                if (item.speed_mps - self.x[1]) ** 2 / variance <= self.config.gate_normal
            ]
            if inliers and len(inliers) < len(candidates):
                self.rejected += len(candidates) - len(inliers)
                self.last_reject = stamp
                candidates = inliers
        measurement = median(item.speed_mps for item in candidates)
        spread = median((item.speed_mps - measurement) ** 2 for item in candidates)
        self._wheel_update(stamp, measurement, max(self.config.wheel_variance_floor, spread))

    def _wheel_update(self, stamp, measurement, variance):
        c = self.config
        if not c.robust:
            self._update(measurement, variance)
            self.last_accept = stamp
            self.accepted += 1
            return
        if stamp - self.last_accept > round(c.wheel_timeout_s * 1e9):
            self.quality, self.streak = "DEGRADED", 0
        jump = False
        if self.z_time is not None and stamp > self.z_time:
            jump = abs(measurement - self.z_prev) > (
                c.max_wheel_accel * (stamp - self.z_time) / 1e9 + 3 * math.sqrt(variance)
            )
        self.z_prev, self.z_time = measurement, stamp
        self.consistent = 0 if jump else self.consistent + 1
        if (stamp - self.last_accept) / 1e9 > c.reacquire_s and self.consistent >= c.recover_updates:
            self._update(measurement, variance)
            self.last_accept = self.last_reacquire = stamp
            self.quality, self.streak = "DEGRADED", 0
            self.accepted += 1
            return
        nis = (measurement - self.x[1]) ** 2 / (min(self.P[1][1], c.gate_sigma_cap**2) + variance)
        if jump or nis > c.gate_reject:
            self.rejected += 1
            self.last_reject = stamp
            self.quality, self.streak = "DEGRADED", 0
        elif nis > c.gate_normal:
            self._update(measurement, variance * nis / c.gate_normal, full=False)
            self.accepted += 1
            self.last_accept = self.last_downweight = stamp
            self.quality, self.streak = "DEGRADED", 0
        else:
            self._update(measurement, variance, full=self.quality == "FUSED")
            self.accepted += 1
            self.last_accept = stamp
            self.streak += 1
            if self.quality == "DEGRADED" and self.streak >= c.recover_updates:
                self.quality = "FUSED"

    def _recent(self, stamp, now):
        return stamp is not None and (now - stamp) / 1e9 <= self.config.reason_hold_s

    def advance_to(self, stamp_ns):
        if stamp_ns < self.t:
            raise ValueError("Clock moved backwards: reset and initialize a new run")
        while self.queue and self.queue[0][0][0] <= stamp_ns:
            group_stamp = self.queue[0][0][0]
            group = []
            while self.queue and self.queue[0][0][0] == group_stamp:
                group.append(heapq.heappop(self.queue)[2])
            self._process_group(group_stamp, group)
        self._predict_to(stamp_ns)
        self.closed_time = stamp_ns
        self.output_seq += 1
        c = self.config
        wheel_samples = {
            key: sample.speed_mps
            for key, sample in self.channels.items()
            if sample.valid
            and key not in self.frozen_ids
            and stamp_ns - sample.stamp_ns <= round(c.wheel_timeout_s * 1e9)
        }
        wheel_age = min(
            (
                (stamp_ns - sample.stamp_ns) / 1e9
                for key, sample in self.channels.items()
                if key in wheel_samples
            ),
            default=None,
        )
        control_age = None if self.last_control is None else (stamp_ns - self.last_control) / 1e9
        control_stale = c.control_timeout_s is not None and (
            control_age is None or control_age > c.control_timeout_s
        )
        duration = (stamp_ns - self.last_accept) / 1e9
        reasons = sorted(self.pending_reasons)
        self.pending_reasons.clear()
        reasons.append("UNCERTAINTY_UNCALIBRATED")
        for marker, code in (
            (self.last_reset, "GAP_RESET"),
            (self.last_reject, "WHEEL_REJECTED"),
            (self.last_downweight, "WHEEL_DOWNWEIGHTED"),
            (self.last_reacquire, "WHEEL_REACQUIRED"),
            (self.last_frozen, "WHEEL_FROZEN"),
        ):
            if self._recent(marker, stamp_ns):
                reasons.append(code)
        if not wheel_samples:
            reasons.append("WHEEL_STALE")
            self.quality, self.streak = "DEGRADED", 0
        if self.last_control is None:
            reasons.append("CONTROL_MISSING")
        elif not self.control_valid:
            reasons.append("CONTROL_INVALID")
        elif control_stale:
            reasons.append("CONTROL_STALE")
        if duration > c.max_model_only_s:
            reasons.append("MODEL_ONLY_HORIZON")
        if self.clamped:
            reasons.append("ZERO_CLAMP")
            self.clamped = False
        finite = all(math.isfinite(item) for item in self.x) and all(
            math.isfinite(item) for row in self.P for item in row
        )
        control_ok = self.last_control is not None and self.control_valid and not control_stale
        if not finite or control_stale or duration > c.max_model_only_s:
            mode, valid = "INVALID", False
        elif wheel_samples:
            mode = "FUSED" if self.quality == "FUSED" and control_ok else "DEGRADED"
            valid = True
        else:
            mode, valid = ("MODEL_ONLY", True) if control_ok else ("INVALID", False)
        covariance = [item for row in self.P for item in row] if finite else None
        velocity = self.x[1] if finite else None
        acceleration = (
            self.x[2] - c.c1 * velocity - c.c2 * velocity * abs(velocity) + self.x[3] if finite else None
        )
        return Estimate(
            stamp_ns=stamp_ns,
            seq=self.output_seq,
            s_m=self.x[0] if finite else None,
            v_mps=velocity,
            mode=mode,
            valid=valid,
            reason_codes=reasons,
            wheel_speeds_mps=wheel_samples,
            control_u=self.control,
            control_age_s=control_age,
            wheel_age_s=wheel_age,
            model_only_duration_s=0.0 if mode == "FUSED" else duration,
            accepted_wheel_count=self.accepted,
            rejected_wheel_count=self.rejected,
            uncertainty_available=finite,
            covariance_4x4=covariance,
            sigma_s_m=math.sqrt(max(self.P[0][0], 0)) if finite else None,
            sigma_v_mps=math.sqrt(max(self.P[1][1], 0)) if finite else None,
            a_mps2=acceleration,
            disturbance_mps2=self.x[3] if finite else None,
            model_version=self.MODEL_VERSION,
        )
