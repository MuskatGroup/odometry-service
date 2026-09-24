"""ROS-independent glue between message dicts and the estimator (importable without ROS).

The rclpy node converts each message with rosidl_runtime_py.convert.message_to_ordereddict and
calls `on_message`. Field paths, units, radii and control encoding come from the same
adapter-1 profile used for files; each channel/control spec names its input `topic`.
Nothing here assumes the organizers' message types.
"""

import copy
import math
from .core import InputError, Profile, integer, normalize
from .live import LiveSession
from .model import ModelConfig, ModelEstimator
from .runner import HoldEstimator

STAMP_FIELD = "_stamp_ns"


def header_stamp_ns(message):
    """header.stamp -> ns, or None when the message has no usable header."""
    try:
        stamp = message["header"]["stamp"]
        return integer(stamp["sec"], "sec") * 1_000_000_000 + integer(stamp["nanosec"], "nanosec")
    except (KeyError, TypeError, InputError):
        return None


class RosBridge:
    """config = {"profile": adapter-1 profile with per-spec "topic", "inputs": [{"topic", "type", "qos"}],
                 "estimator": {"type": "model"|"hold", "config": {...}},
                 "stamp_source": "header"|"receive", "reorder_ms": 20,
                 "output": {"frame_id", "child_frame_id", "rate_hz"}}"""

    def __init__(self, config: dict, journal=None):
        prof = copy.deepcopy(config["profile"])
        prof.setdefault("timestamp", {"field": STAMP_FIELD, "unit": "ns"})
        self.topics = [item["topic"] for item in config["inputs"]]
        if len(set(self.topics)) != len(self.topics):
            raise InputError("Duplicate input topic")
        default = self.topics[0] if len(self.topics) == 1 else None
        for spec in [*prof.get("channels", []), *([prof["control"]] if prof.get("control") else [])]:
            spec.setdefault("topic", default)
            if spec["topic"] not in self.topics:
                raise InputError(f"Spec {spec.get('id', 'control')} needs a topic among {self.topics}")
        self.profile = Profile(prof)
        self.stamp_source = config.get("stamp_source", "header")
        if self.stamp_source not in ("header", "receive"):
            raise InputError("stamp_source must be header or receive")
        self.reorder_ns = int(float(config.get("reorder_ms", 20)) * 1e6)
        if self.reorder_ns < 0:
            raise InputError("reorder_ms must be nonnegative")
        self.output = {"frame_id": "odom_1d", "child_frame_id": "tram_1d", "rate_hz": 50.0,
                       **config.get("output", {})}
        self.subprofiles, self.counters = {}, {}
        for topic in self.topics:
            sub = copy.deepcopy(prof)
            sub["channels"] = [{k: v for k, v in c.items() if k != "derived_from"}
                               for c in prof.get("channels", []) if c["topic"] == topic]
            if not (prof.get("control") and prof["control"]["topic"] == topic):
                sub.pop("control", None)
            sub.pop("fusion", None)
            self.subprofiles[topic] = Profile(sub)
            self.counters[topic] = {}
        setting = config.get("estimator", {"type": "model"})
        if setting.get("type", "model") == "model":
            estimator = ModelEstimator(self.profile, ModelConfig.from_dict(setting.get("config", {})))
        elif setting["type"] == "hold":
            estimator = HoldEstimator(self.profile, **setting.get("config", {}))
        else:
            raise InputError("estimator.type must be model or hold")
        self.session = LiveSession(self.profile, estimator=estimator, journal=journal)
        self.dropped = 0

    def on_message(self, topic, message, receive_ns):
        """Normalize one message and queue it. Returns the number of events queued."""
        if topic not in self.subprofiles:
            raise InputError(f"Unconfigured topic: {topic}")
        stamp = header_stamp_ns(message) if self.stamp_source == "header" else None
        record = dict(message)
        record[STAMP_FIELD] = receive_ns if stamp is None else stamp
        count = 0
        for batch in normalize([record], self.subprofiles[topic], self.counters[topic]):
            self.session.ingest(batch)
            count += len(batch)
        return count

    def tick(self, clock_ns):
        """Advance estimator time (call from an independent timer, also during silence).

        The estimator is run `reorder_ms` behind the clock so slightly late header stamps
        are still ordered instead of dropped. Returns a frame, or None if time did not advance.
        """
        target = clock_ns - self.reorder_ns
        if self.session.closed is not None and target <= self.session.closed:
            return None
        return self.session.advance_to(target)

    def odometry(self, frame):
        """Fields for nav_msgs/Odometry, or None while there is no valid finite estimate."""
        if not frame["valid"] or frame["s_m"] is None:
            return None
        cov = frame["covariance_4x4"]
        values = [frame["s_m"], frame["v_mps"], cov[0], cov[5]] if cov else None
        if values is None or not all(math.isfinite(v) for v in values):
            return None
        return {"frame_id": self.output["frame_id"], "child_frame_id": self.output["child_frame_id"],
                "stamp_ns": int(frame["stamp_ns"]), "x": values[0], "vx": values[1],
                "var_x": values[2], "var_vx": values[3]}
