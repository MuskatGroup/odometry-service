"""Transport-independent domain types. SI units, integer event time."""

from dataclasses import asdict, dataclass, field
from typing import Protocol


@dataclass(frozen=True)
class ControlSample:
    stamp_ns: int
    seq: int
    u: float
    valid: bool = True


@dataclass(frozen=True)
class WheelSample:
    stamp_ns: int
    seq: int
    wheel_id: str
    speed_mps: float
    valid: bool = True


Event = ControlSample | WheelSample


@dataclass(frozen=True)
class InitialState:
    stamp_ns: int
    s_m: float = 0.0
    v_mps: float = 0.0
    covariance_4x4: tuple[float, ...] | None = None


@dataclass(frozen=True)
class EstimatorConfig:
    wheel_timeout_s: float = 0.5
    control_timeout_s: float = 0.5
    max_hold_s: float = 2.0
    max_channels: int = 64
    max_queue: int = 4096
    dedup_capacity: int = 8192


@dataclass
class Estimate:
    stamp_ns: int
    seq: int
    s_m: float | None
    v_mps: float | None
    mode: str
    valid: bool
    reason_codes: list[str]
    wheel_speeds_mps: dict[str, float] = field(default_factory=dict)
    control_u: float | None = None
    control_age_s: float | None = None
    wheel_age_s: float | None = None
    model_only_duration_s: float = 0.0
    accepted_wheel_count: int = 0
    rejected_wheel_count: int = 0
    uncertainty_available: bool = False
    covariance_4x4: list[float] | None = None
    sigma_s_m: float | None = None
    sigma_v_mps: float | None = None
    a_mps2: float | None = None
    disturbance_mps2: float | None = None
    model_version: str = "wheel-hold-v1"

    def to_dict(self) -> dict:
        result = asdict(self)
        result["stamp_ns"] = str(self.stamp_ns)
        result["has_estimate"] = self.s_m is not None and self.v_mps is not None
        return result


def event_dict(event: Event) -> dict:
    result = asdict(event)
    result["stamp_ns"] = str(event.stamp_ns)
    result["kind"] = "control" if isinstance(event, ControlSample) else "wheel"
    return result


def event_key(event: Event) -> tuple:
    return (
        event.stamp_ns,
        0 if isinstance(event, ControlSample) else 1,
        "" if isinstance(event, ControlSample) else event.wheel_id,
        event.seq,
    )


class Estimator(Protocol):
    def initialize(self, initial: InitialState, config: EstimatorConfig) -> None: ...
    def ingest_control(self, sample: ControlSample) -> None: ...
    def ingest_wheel(self, sample: WheelSample) -> None: ...
    def advance_to(self, stamp_ns: int) -> Estimate: ...
    def reset(self) -> None: ...
