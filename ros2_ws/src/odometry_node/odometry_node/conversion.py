from odometry_core import ControlSample, WheelSample
from odometry_msgs.msg import ControlSample as ControlMsg
from odometry_msgs.msg import LongitudinalEstimate
from odometry_msgs.msg import WheelSample as WheelMsg


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(header, stamp):
    header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return header


def event_message(event):
    msg = ControlMsg() if isinstance(event, ControlSample) else WheelMsg()
    set_stamp(msg.header, event.stamp_ns)
    msg.seq, msg.valid = event.seq, event.valid
    if isinstance(event, ControlSample):
        msg.u = float(event.u)
    else:
        msg.wheel_id, msg.speed_mps = event.wheel_id, float(event.speed_mps)
    return msg


def domain_event(msg):
    if isinstance(msg, ControlMsg):
        return ControlSample(ns(msg.header.stamp), msg.seq, msg.u, msg.valid)
    return WheelSample(ns(msg.header.stamp), msg.seq, msg.wheel_id, msg.speed_mps, msg.valid)


def estimate_message(estimate, run_id, compute_ms):
    msg = LongitudinalEstimate()
    set_stamp(msg.header, estimate.stamp_ns)
    msg.header.frame_id = "odom_1d"
    msg.run_id, msg.seq = run_id, estimate.seq
    msg.has_estimate = estimate.s_m is not None and estimate.v_mps is not None
    msg.s_m, msg.v_mps = float(estimate.s_m or 0), float(estimate.v_mps or 0)
    msg.mode, msg.valid, msg.reason_codes = estimate.mode, estimate.valid, estimate.reason_codes
    msg.uncertainty_available = estimate.uncertainty_available
    msg.covariance_4x4 = estimate.covariance_4x4 or [0.0] * 16
    msg.sigma_s_m, msg.sigma_v_mps = float(estimate.sigma_s_m or 0), float(estimate.sigma_v_mps or 0)
    msg.wheel_ids = list(estimate.wheel_speeds_mps)
    msg.wheel_speeds_mps = list(estimate.wheel_speeds_mps.values())
    msg.has_control = estimate.control_u is not None
    msg.control_u = float(estimate.control_u or 0)
    for field in ("control_age", "wheel_age"):
        value = getattr(estimate, field + "_s")
        setattr(msg, "has_" + field, value is not None)
        setattr(msg, field + "_s", float(value or 0))
    msg.model_only_duration_s = estimate.model_only_duration_s
    msg.accepted_wheel_count = estimate.accepted_wheel_count
    msg.rejected_wheel_count = estimate.rejected_wheel_count
    msg.model_version, msg.compute_ms = estimate.model_version, compute_ms
    return msg


def frame_dict(msg):
    return {
        "schema_version": "0.2",
        "run_id": msg.run_id,
        "seq": msg.seq,
        "stamp_ns": str(ns(msg.header.stamp)),
        "has_estimate": msg.has_estimate,
        "s_m": msg.s_m if msg.has_estimate else None,
        "v_mps": msg.v_mps if msg.has_estimate else None,
        "mode": msg.mode,
        "valid": msg.valid,
        "reason_codes": list(msg.reason_codes),
        "uncertainty_available": msg.uncertainty_available,
        "sigma_s_m": msg.sigma_s_m if msg.uncertainty_available else None,
        "sigma_v_mps": msg.sigma_v_mps if msg.uncertainty_available else None,
        "covariance_4x4": list(msg.covariance_4x4) if msg.uncertainty_available else None,
        "wheel_speeds_mps": dict(zip(msg.wheel_ids, msg.wheel_speeds_mps)),
        "control_u": msg.control_u if msg.has_control else None,
        "control_age_s": msg.control_age_s if msg.has_control_age else None,
        "wheel_age_s": msg.wheel_age_s if msg.has_wheel_age else None,
        "compute_ms": msg.compute_ms,
        "model_version": msg.model_version,
        "model_only_duration_s": msg.model_only_duration_s,
        "accepted_wheel_count": msg.accepted_wheel_count,
        "rejected_wheel_count": msg.rejected_wheel_count,
    }
