"""Contract v0.1 facade (docs/03): initialize / ingest_control / ingest_wheel / advance_to / reset.

Built on LiveSession (bounded queue, late/duplicate handling) + ModelEstimator.
Nothing here touches files, HTTP or ROS.
"""

from dataclasses import asdict, dataclass, field
from .core import Event, InputError, Profile
from .live import LiveSession
from .model import ModelConfig, ModelEstimator

SOURCE_ID = "core"


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
class InitialState:
    stamp_ns: int
    s_m: float
    v_mps: float
    covariance_4x4: list | None = None  # row-major nested 4x4, order [s, v, a_act, d]


@dataclass(frozen=True)
class EstimatorConfig:
    wheel_ids: tuple[str, ...]
    model: ModelConfig = field(default_factory=ModelConfig)
    fusion: dict | None = None
    max_pending: int = 10000
    dedup_capacity: int = 100000


@dataclass(frozen=True)
class Estimate:
    stamp_ns: int
    seq: int
    s_m: float | None
    v_mps: float | None
    a_mps2: float | None
    disturbance_mps2: float | None
    covariance_4x4: list | None
    sigma_s_m: float | None
    sigma_v_mps: float | None
    mode: str
    valid: bool
    reason_codes: list
    control_age_s: float | None
    wheel_age_s: float | None
    model_only_duration_s: float
    accepted_wheel_count: int
    rejected_wheel_count: int
    model_version: str

    @classmethod
    def from_frame(cls, frame):
        known = cls.__dataclass_fields__
        data = {k: v for k, v in frame.items() if k in known}
        data["stamp_ns"] = int(data["stamp_ns"])
        return cls(**data)

    def to_dict(self):
        result = asdict(self)
        result["stamp_ns"] = str(self.stamp_ns)  # JSON-safe per contract
        return result


class OdometryEstimator:
    def __init__(self, config: EstimatorConfig):
        if not config.wheel_ids or len(set(config.wheel_ids)) != len(config.wheel_ids):
            raise InputError("wheel_ids must be nonempty and unique")
        self.config = config
        fusion = {"strategy": "median"}
        fusion.update(config.fusion or {})
        self.profile = Profile({
            "schema_version": "adapter-1", "source_id": SOURCE_ID,
            "timestamp": {"field": "t", "unit": "ns"},
            "channels": [{"id": w, "field": w, "unit": "m/s"} for w in config.wheel_ids],
            "control": {"field": "u"}, "fusion": fusion})
        self.reset()

    def reset(self):
        self.model = ModelEstimator(self.profile, self.config.model)
        self.session = LiveSession(self.profile, estimator=self.model,
                                   max_pending=self.config.max_pending,
                                   dedup_capacity=self.config.dedup_capacity)

    def initialize(self, initial: InitialState, config: EstimatorConfig | None = None):
        """Call before feeding events; the initial stamp must not exceed the first event."""
        if config is not None:
            self.config = config
        self.reset()
        self.model.set_initial(initial.stamp_ns, initial.s_m, initial.v_mps, initial.covariance_4x4)

    def ingest_control(self, sample: ControlSample):
        value = sample.u if sample.valid else None
        self.session.ingest([Event(sample.stamp_ns, SOURCE_ID, sample.seq, "control", None, value, sample.valid)])

    def ingest_wheel(self, sample: WheelSample):
        if sample.wheel_id not in self.config.wheel_ids:
            raise InputError(f"Unknown wheel_id: {sample.wheel_id}")
        value = sample.speed_mps if sample.valid else None
        self.session.ingest([Event(sample.stamp_ns, SOURCE_ID, sample.seq, "speed", sample.wheel_id,
                                   value, sample.valid)])

    def advance_to(self, stamp_ns: int) -> Estimate:
        return Estimate.from_frame(self.session.advance_to(stamp_ns))
