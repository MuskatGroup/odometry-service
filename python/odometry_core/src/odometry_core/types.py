"""Transport-independent domain types. SI units, integer event time."""

from dataclasses import asdict, dataclass, field
from enum import Enum
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


@dataclass(frozen=True)
class LongitudinalVelocityCorrection:
    stamp_ns: int
    seq: int
    v_mps: float
    variance_m2ps2: float
    source: str


@dataclass(frozen=True)
class AlongTrackPositionCorrection:
    stamp_ns: int
    seq: int
    route_id: str
    s_m: float
    variance_m2: float
    source: str


class WheelHealthState(str, Enum):
    NORMAL = "normal"
    POSITIVE_SLIP = "positive_slip"
    BRAKING_SLIDE = "braking_slide"
    FROZEN = "frozen"
    DROPOUT = "dropout"
    INCONSISTENT = "inconsistent"
    UNKNOWN = "unknown"


Event = (
    ControlSample
    | WheelSample
    | LongitudinalVelocityCorrection
    | AlongTrackPositionCorrection
)


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
    wheel_health: dict[str, str] = field(default_factory=dict)
    route_id: str | None = None
    accepted_gnss_velocity_count: int = 0
    rejected_gnss_velocity_count: int = 0
    accepted_gnss_position_count: int = 0
    rejected_gnss_position_count: int = 0
    model_version: str = "wheel-hold-v1"

    def to_dict(self) -> dict:
        result = asdict(self)
        result["stamp_ns"] = str(self.stamp_ns)
        result["has_estimate"] = self.s_m is not None and self.v_mps is not None
        return result


def event_dict(event: Event) -> dict:
    result = asdict(event)
    result["stamp_ns"] = str(event.stamp_ns)
    if isinstance(event, ControlSample):
        result["kind"] = "control"
    elif isinstance(event, WheelSample):
        result["kind"] = "wheel"
    elif isinstance(event, LongitudinalVelocityCorrection):
        result["kind"] = "velocity_correction"
    else:
        result["kind"] = "position_correction"
    return result


def event_key(event: Event) -> tuple:
    if isinstance(event, ControlSample):
        order, identifier = 0, ""
    elif isinstance(event, WheelSample):
        order, identifier = 1, event.wheel_id
    elif isinstance(event, LongitudinalVelocityCorrection):
        order, identifier = 2, event.source
    else:
        order, identifier = 3, event.source
    return (
        event.stamp_ns,
        order,
        identifier,
        event.seq,
    )


class Estimator(Protocol):
    def initialize(self, initial: InitialState, config: EstimatorConfig) -> None: ...
    def ingest_control(self, sample: ControlSample) -> None: ...
    def ingest_wheel(self, sample: WheelSample) -> None: ...
    def ingest_velocity_correction(self, sample: LongitudinalVelocityCorrection) -> None: ...
    def ingest_position_correction(self, sample: AlongTrackPositionCorrection) -> None: ...
    def advance_to(self, stamp_ns: int) -> Estimate: ...
    def reset(self) -> None: ...
