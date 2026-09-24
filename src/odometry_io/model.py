"""Four-state EKF [s, v, a_act, d] behind the Estimator interface (docs/01, docs/03).

Pure Python, no ROS. All parameters are experimental starting values until real
data exists; sigma comes from the covariance but its coverage is NOT yet calibrated.
"""

from dataclasses import asdict, dataclass, fields
import math
from .core import Event, InputError, Profile, number
from .fusion import STRATEGIES

N = 4


@dataclass(frozen=True)
class ModelConfig:
    # Drive/brake model: a_cmd = kt*max(u-u_dead,0)/(1+v/v_sat) - kb*max(-u-u_dead,0)*min(1,v/v_brake)
    kt: float = 1.2
    kb: float = 1.5
    u_dead: float = 0.05
    v_sat: float = 40.0
    v_brake: float = 0.5
    tau_s: float = 0.3
    c1: float = 0.01
    c2: float = 0.0
    # Process noise (variance per second) and initial covariance diagonal.
    q_v: float = 0.01
    q_a: float = 0.05
    q_d: float = 0.001
    p0_s: float = 1.0
    p0_v: float = 0.25
    p0_a: float = 1.0
    p0_d: float = 0.25
    # Wheel trust: normalized innovation squared gates, jump check, recovery hysteresis.
    wheel_variance_floor: float = 0.04
    gate_normal: float = 9.0
    gate_reject: float = 36.0
    max_wheel_accel: float = 6.0
    # The gate uses min(P_vv, gate_sigma_cap^2): without the cap, covariance growth during a
    # sustained wheel fault opens the gate and the estimate collapses onto the faulty wheels.
    gate_sigma_cap: float = 1.0
    # After reacquire_s without an accepted update, a run of self-consistent wheel samples
    # re-seeds v from the wheels (the model may have drifted); a persistent common slip that
    # long is indistinguishable from real motion.
    reacquire_s: float = 10.0
    recover_updates: int = 5
    # Frozen sensor: a channel repeating one exact value for freeze_s while the control says the
    # tram should be accelerating/braking (|open-loop accel| >= freeze_min_accel) is dropped.
    # Steady coasting with quantized readings is not flagged because the control expects no change.
    detect_freeze: bool = True
    # Per-channel vote: with >= 2 candidates, channels whose innovation exceeds gate_normal are
    # dropped when at least one channel agrees with the model prediction (a median cannot do
    # this with 2 wheels, or with several slipping ones).
    channel_gate: bool = True
    freeze_s: float = 1.0
    freeze_min_accel: float = 0.2
    reason_hold_s: float = 1.0
    # Timing/quality horizons. control_timeout_s=None holds the last control forever
    # (knobs may publish only on change); set it once the real cadence is known.
    wheel_timeout_s: float = 0.5
    control_timeout_s: float | None = None
    max_model_only_s: float = 10.0
    max_step_s: float = 0.02
    max_gap_s: float = 60.0
    # Ablation switches for the docs/05 baselines: use_wheels=False is B1 (model-only after
    # initialization), robust=False is B2 (plain EKF, no gating). adapt_disturbance=True is M2; it
    # is off by default because on the synthetic train/holdout benchmark the fixed-d filter (M1) had
    # the lower total error (adaptation helped in some scenarios and hurt in others). Re-decide on real data.
    use_wheels: bool = True
    robust: bool = True
    adapt_disturbance: bool = False

    def __post_init__(self):
        for f in fields(self):
            if f.type is bool:
                if type(getattr(self, f.name)) is not bool:
                    raise InputError(f"{f.name} must be true or false")
            elif f.name != "control_timeout_s" or getattr(self, f.name) is not None:
                number(getattr(self, f.name), f.name)
        positive = ("kt", "kb", "v_sat", "v_brake", "tau_s", "wheel_variance_floor", "gate_normal",
                    "gate_reject", "max_wheel_accel", "gate_sigma_cap", "reacquire_s", "freeze_s", "freeze_min_accel", "wheel_timeout_s", "max_model_only_s",
                    "max_step_s", "max_gap_s", "p0_s", "p0_v", "p0_a", "p0_d")
        nonneg = ("u_dead", "c1", "c2", "q_v", "q_a", "q_d", "reason_hold_s")
        if any(getattr(self, name) <= 0 for name in positive) or any(getattr(self, name) < 0 for name in nonneg):
            raise InputError("Invalid model parameter range")
        if self.gate_reject < self.gate_normal or self.recover_updates < 1:
            raise InputError("gate_reject must be >= gate_normal and recover_updates >= 1")
        if self.control_timeout_s is not None and self.control_timeout_s <= 0:
            raise InputError("control_timeout_s must be positive or null")

    @classmethod
    def from_dict(cls, data):
        known = {f.name for f in fields(cls)}
        unknown = set(data) - known
        if unknown:
            raise InputError(f"Unknown model parameters: {sorted(unknown)}")
        return cls(**data)


def _mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(N)) for j in range(N)] for i in range(N)]


def _t(a):
    return [[a[j][i] for j in range(N)] for i in range(N)]


def _finite(values):
    return all(math.isfinite(v) for v in values)


class ModelEstimator:
    """EKF with robust wheel gating. Time moves only via ingest()/advance_to()."""

    MODEL_VERSION = "ekf-1"

    def __init__(self, profile: Profile, config: ModelConfig | None = None):
        self.profile = profile
        self.config = config or ModelConfig()
        strategy = profile.fusion.get("strategy", "median")
        if strategy not in STRATEGIES:
            raise InputError(f"Unknown fusion strategy: {strategy}")
        if strategy == "selected_channel" and profile.fusion.get("channel") not in profile.selected:
            raise InputError("selected_channel requires a selected configured channel")
        self.strategy = STRATEGIES[strategy]
        self.fusion_settings = dict(profile.fusion)
        if "variance_floor" not in profile.config.get("fusion", {}):
            self.fusion_settings["variance_floor"] = self.config.wheel_variance_floor
        self.timeout_ns = int(self.config.wheel_timeout_s * 1e9)
        self.channels = {}
        self.sequence = 0
        self.time = None
        self.control = None
        self.control_valid = True
        self.last_control = None
        self.last_used = ()
        self.accepted = self.rejected = 0
        self.last_reject = None
        self.last_downweight = None
        self.last_reacquire = None
        self.last_frozen = None
        self.frozen_ids = set()
        self.hold = {}
        self.last_reset = None
        self.clamped = False
        self.carry_s = 0.0  # position kept across a GAP_RESET; its sigma restarts at p0_s
        self._clear_filter()

    def describe(self):
        return {"model_version": self.MODEL_VERSION, **asdict(self.config)}

    def _clear_filter(self):
        self.initialized = False
        self.x = [0.0] * N
        self.P = [[0.0] * N for _ in range(N)]
        self.last_accept = None
        self.quality = "FUSED"
        self.streak = 0
        self.z_prev = self.z_time = None
        self.consistent = 0

    def _init_state(self, stamp, s, v, cov):
        c = self.config
        self.x = [s, max(v, 0.0), 0.0, 0.0]
        self.P = cov if cov is not None else [[0.0] * N for _ in range(N)]
        if cov is None:
            # adapt_disturbance=False keeps d fixed at 0: no variance, no growth (M1/B2 baselines).
            for i, p in enumerate((c.p0_s, c.p0_v, c.p0_a, c.p0_d if c.adapt_disturbance else 1e-12)):
                self.P[i][i] = p
        self.initialized = True
        self.last_accept = stamp
        self.quality, self.streak = "FUSED", 0

    def set_initial(self, stamp_ns, s_m, v_mps, covariance=None):
        """Explicit initial condition (contract InitialState); must precede events."""
        if not _finite([number(s_m, "s_m"), number(v_mps, "v_mps")]):
            raise InputError("Initial state must be finite")
        if covariance is not None:
            covariance = [[number(covariance[i][j], "covariance") for j in range(N)] for i in range(N)]
        self._predict_to(stamp_ns)
        self._init_state(stamp_ns, s_m, v_mps, covariance)

    # -- model ------------------------------------------------------------
    def _command(self, u, v):
        c = self.config
        drive, brake = max(u - c.u_dead, 0.0), max(-u - c.u_dead, 0.0)
        g = 1.0 / (1.0 + v / c.v_sat)
        b, db = (min(1.0, v / c.v_brake), 1.0 / c.v_brake if v < c.v_brake else 0.0) if v > 0 else (0.0, 1.0 / c.v_brake)
        return (c.kt * drive * g - c.kb * brake * b,
                -c.kt * drive * g * g / c.v_sat - c.kb * brake * db)

    def _step(self, dt, u):
        c = self.config
        s, v, a, d = self.x
        ex = math.exp(-dt / c.tau_s)
        alpha = 1.0 - ex
        acmd, dacmd = self._command(u, v)
        ddrag = c.c1 + 2 * c.c2 * abs(v)
        new = [s + v * dt,
               v + (a - c.c1 * v - c.c2 * v * abs(v) + d) * dt,
               a + (acmd - a) * alpha,
               d]
        F = [[1, dt, 0, 0], [0, 1 - ddrag * dt, dt, dt], [0, alpha * dacmd, ex, 0], [0, 0, 0, 1]]
        P = _mm(_mm(F, self.P), _t(F))
        for i, q in ((1, c.q_v), (2, c.q_a), (3, c.q_d if c.adapt_disturbance else 0.0)):
            P[i][i] += q * dt
        if new[1] < 0.0:
            new[1] = 0.0
            self.clamped = True
        self.x, self.P = new, P

    def _predict_to(self, stamp):
        if self.time is None:
            self.time = stamp
            return
        if stamp < self.time:
            raise InputError("Estimator time cannot move backwards")
        total = (stamp - self.time) / 1e9
        if total > 0 and self.initialized:
            if total > self.config.max_gap_s:
                self.carry_s = self.x[0]
                self._clear_filter()
                self.last_reset = stamp
            else:
                n = max(1, math.ceil(total / self.config.max_step_s))
                u = self.control if self.control is not None else 0.0
                for _ in range(n):
                    self._step(total / n, u)
        self.time = stamp

    # -- measurement ------------------------------------------------------
    def _update(self, z, r, full):
        P = self.P
        s_inn = P[1][1] + r
        gain = [P[i][1] / s_inn for i in range(N)]
        if not full:
            gain[2] = gain[3] = 0.0
        if not self.config.adapt_disturbance:
            gain[3] = 0.0
        e = z - self.x[1]
        self.x = [self.x[i] + gain[i] * e for i in range(N)]
        ikh = [[(1.0 if i == j else 0.0) - (gain[i] if j == 1 else 0.0) for j in range(N)] for i in range(N)]
        new = _mm(_mm(ikh, P), _t(ikh))
        for i in range(N):
            for j in range(N):
                new[i][j] += gain[i] * r * gain[j]
        for i in range(N):
            for j in range(i):
                new[i][j] = new[j][i] = 0.5 * (new[i][j] + new[j][i])
            new[i][i] = max(new[i][i], 1e-12)
        self.P = new
        self.x[1] = max(self.x[1], 0.0)

    def _wheel_update(self, stamp, z, r):
        c = self.config
        if not self.initialized:
            self._init_state(stamp, self.carry_s, z, None)
            self.z_prev, self.z_time = z, stamp
            self.accepted += 1
            return
        if not c.use_wheels:
            return
        if not c.robust:
            self._update(z, r, full=True)
            self.accepted += 1
            self.last_accept = stamp
            return
        if stamp - self.last_accept > self.timeout_ns:
            self.quality, self.streak = "DEGRADED", 0
        jump = False
        if self.z_time is not None and stamp > self.z_time:
            dz = abs(z - self.z_prev)
            jump = dz > c.max_wheel_accel * (stamp - self.z_time) / 1e9 + 3.0 * math.sqrt(r)
        self.z_prev, self.z_time = z, stamp
        self.consistent = 0 if jump else self.consistent + 1
        if (stamp - self.last_accept) / 1e9 > c.reacquire_s and self.consistent >= c.recover_updates:
            # A normal full update that bypasses the gate: the correlations in P also correct a_act/d,
            # which overwriting v alone would leave stale.
            self._update(z, r, full=True)
            self.last_accept = self.last_reacquire = stamp
            self.quality, self.streak = "DEGRADED", 0
            self.accepted += 1
            return
        nis = (z - self.x[1]) ** 2 / (min(self.P[1][1], c.gate_sigma_cap ** 2) + r)
        if jump or nis > c.gate_reject:
            self.rejected += 1
            self.last_reject = stamp
            self.quality, self.streak = "DEGRADED", 0
        elif nis > c.gate_normal:
            self._update(z, r * nis / c.gate_normal, full=False)
            self.accepted += 1
            self.last_accept = self.last_downweight = stamp
            self.quality, self.streak = "DEGRADED", 0
        else:
            self._update(z, r, full=self.quality == "FUSED")
            self.accepted += 1
            self.last_accept = stamp
            self.streak += 1
            if self.quality == "DEGRADED" and self.streak >= c.recover_updates:
                self.quality = "FUSED"

    # -- Estimator interface ----------------------------------------------
    def ingest(self, stamp_ns, events: list[Event]):
        self._predict_to(stamp_ns)
        configured = {spec["id"] for spec in self.profile.channels}
        candidates, streams = [], set()
        for event in events:
            stream = event.kind, event.channel_id
            if stream in streams:
                raise InputError("Multiple new values for one stream at the same timestamp")
            streams.add(stream)
            if event.kind == "control":
                self.control_valid = event.input_valid
                self.control = event.value if event.input_valid else None
                self.last_control = event.stamp_ns
            else:
                if event.channel_id not in configured:
                    raise InputError(f"Unknown normalized channel: {event.channel_id}")
                self.channels[event.channel_id] = event
                if event.input_valid and event.channel_id in self.profile.selected:
                    if self._is_frozen(event):
                        self.frozen_ids.add(event.channel_id)
                        self.last_frozen = stamp_ns
                    else:
                        self.frozen_ids.discard(event.channel_id)
                        candidates.append(event)
        candidates = self._vote(stamp_ns, candidates)
        result = self.strategy(candidates, self.fusion_settings) if candidates else None
        if result is not None:
            z = number(result.speed_mps, "fused speed")
            r = number(result.variance, "fused variance")
            if r <= 0:
                raise InputError("Fusion variance must be positive")
            self.last_used = result.used_channels
            self._wheel_update(stamp_ns, z, r)

    def _vote(self, stamp, candidates):
        c = self.config
        if not (c.robust and c.channel_gate and self.initialized and len(candidates) >= 2):
            return candidates
        variance = self.fusion_settings["variance_floor"] + min(self.P[1][1], c.gate_sigma_cap ** 2)
        inliers = [e for e in candidates if (e.value - self.x[1]) ** 2 / variance <= c.gate_normal]
        if not inliers or len(inliers) == len(candidates):
            return candidates  # nothing agrees with the model: leave it to the fused gate
        self.rejected += len(candidates) - len(inliers)
        self.last_reject = stamp
        return inliers

    def _is_frozen(self, event):
        c = self.config
        last = self.hold.get(event.channel_id)
        if last is None or last[0] != event.value:
            self.hold[event.channel_id] = [event.value, event.stamp_ns]
            return False
        if not c.detect_freeze or not self.initialized or (event.stamp_ns - last[1]) / 1e9 < c.freeze_s:
            return False
        v = self.x[1]
        u = self.control if self.control is not None else 0.0
        expected = self._command(u, v)[0] - c.c1 * v - c.c2 * v * abs(v)
        return abs(expected) >= c.freeze_min_accel

    def _fresh_wheels(self, stamp):
        return [e for e in (self.channels[ch] for ch in self.last_used if ch in self.channels)
                if e.input_valid and stamp - e.stamp_ns <= self.timeout_ns and e.channel_id not in self.frozen_ids]

    def _recent(self, then, stamp):
        return then is not None and (stamp - then) / 1e9 <= self.config.reason_hold_s

    def advance_to(self, stamp_ns):
        self._predict_to(stamp_ns)
        c = self.config
        seq, self.sequence = self.sequence, self.sequence + 1
        control_age = None if self.last_control is None else (stamp_ns - self.last_control) / 1e9
        fresh = self._fresh_wheels(stamp_ns) if self.initialized else []
        wheel_age = min(((stamp_ns - e.stamp_ns) / 1e9 for e in fresh), default=None)
        reasons = ["UNCERTAINTY_UNCALIBRATED"]
        clamped, self.clamped = self.clamped, False
        if self._recent(self.last_reset, stamp_ns):
            reasons.append("GAP_RESET")
        finite = self.initialized and _finite(self.x) and all(_finite(row) for row in self.P)
        if not self.initialized or not finite:
            if self.initialized:
                reasons.append("NUMERIC")
                self._clear_filter()
            return self._frame(stamp_ns, seq, "INITIALIZING", False, reasons, None, control_age, None, 0.0)

        control_stale = c.control_timeout_s is not None and control_age is not None and control_age > c.control_timeout_s
        control_ok = self.last_control is not None and self.control_valid and not control_stale
        if self.last_control is None:
            reasons.append("CONTROL_MISSING")
        elif not self.control_valid:
            reasons.append("CONTROL_INVALID")
        elif control_stale:
            reasons.append("CONTROL_STALE")
        duration = (stamp_ns - self.last_accept) / 1e9
        if not fresh:
            reasons.append("WHEEL_STALE")
            self.quality, self.streak = "DEGRADED", 0
        if self._recent(self.last_reject, stamp_ns):
            reasons.append("WHEEL_REJECTED")
        if self._recent(self.last_downweight, stamp_ns):
            reasons.append("WHEEL_DOWNWEIGHTED")
        if self._recent(self.last_reacquire, stamp_ns):
            reasons.append("WHEEL_REACQUIRED")
        if self.frozen_ids or self._recent(self.last_frozen, stamp_ns):
            reasons.append("WHEEL_FROZEN")
        if clamped:
            reasons.append("ZERO_CLAMP")
        if control_stale or duration > c.max_model_only_s:
            if duration > c.max_model_only_s:
                reasons.append("MODEL_ONLY_HORIZON")
            mode = "INVALID"
        elif fresh:
            mode = "FUSED" if self.quality == "FUSED" and control_ok else "DEGRADED"
        else:
            mode = "MODEL_ONLY" if control_ok else "INVALID"
        return self._frame(stamp_ns, seq, mode, mode != "INVALID", reasons, self.x, control_age, wheel_age,
                           0.0 if mode == "FUSED" else duration)

    def _frame(self, stamp_ns, seq, mode, valid, reasons, x, control_age, wheel_age, duration):
        c = self.config
        have = x is not None
        v = x[1] if have else None
        accel = x[2] - c.c1 * v - c.c2 * v * abs(v) + x[3] if have else None
        return {
            "stamp_ns": str(stamp_ns), "seq": seq,
            "s_m": x[0] if have else None, "v_mps": v,
            "a_mps2": accel, "disturbance_mps2": x[3] if have else None,
            "covariance_4x4": [p for row in self.P for p in row] if have else None,
            "sigma_s_m": math.sqrt(self.P[0][0]) if have else None,
            "sigma_v_mps": math.sqrt(self.P[1][1]) if have else None,
            "mode": mode, "valid": valid, "reason_codes": reasons,
            "control_u": self.control, "control_age_s": control_age, "wheel_age_s": wheel_age,
            "model_only_duration_s": duration,
            "accepted_wheel_count": self.accepted, "rejected_wheel_count": self.rejected,
            "model_version": self.MODEL_VERSION,
            "speed_channels": [{"channel_id": key, "speed_mps": e.value, "stamp_ns": str(e.stamp_ns),
                                "input_valid": e.input_valid, "age_s": (stamp_ns - e.stamp_ns) / 1e9}
                               for key, e in sorted(self.channels.items())],
        }
