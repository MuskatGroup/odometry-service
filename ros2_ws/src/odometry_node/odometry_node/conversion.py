from odometry_msgs.msg import LongitudinalEstimate


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def set_stamp(header, stamp):
    header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    return header


def estimate_message(estimate, run_id, compute_ms, map_pose=None, gnss_policy="disabled", gnss_age_s=None):
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
    msg.has_acceleration = estimate.a_mps2 is not None
    msg.a_mps2 = float(estimate.a_mps2 or 0)
    msg.has_disturbance = estimate.disturbance_mps2 is not None
    msg.disturbance_mps2 = float(estimate.disturbance_mps2 or 0)
    msg.wheel_ids = list(estimate.wheel_speeds_mps)
    msg.wheel_speeds_mps = list(estimate.wheel_speeds_mps.values())
    msg.wheel_health_ids = sorted(estimate.wheel_health)
    msg.wheel_health_states = [estimate.wheel_health[key] for key in msg.wheel_health_ids]
    msg.has_control = estimate.control_u is not None
    msg.control_u = float(estimate.control_u or 0)
    for field in ("control_age", "wheel_age"):
        value = getattr(estimate, field + "_s")
        setattr(msg, "has_" + field, value is not None)
        setattr(msg, field + "_s", float(value or 0))
    msg.model_only_duration_s = estimate.model_only_duration_s
    msg.accepted_wheel_count = estimate.accepted_wheel_count
    msg.rejected_wheel_count = estimate.rejected_wheel_count
    msg.route_id = estimate.route_id or ""
    msg.has_map_pose = map_pose is not None
    if map_pose is not None:
        msg.map_x_m = float(map_pose.x_m)
        msg.map_y_m = float(map_pose.y_m)
        msg.map_z_m = float(map_pose.z_m)
        msg.yaw_rad = float(map_pose.yaw_rad)
    msg.gnss_policy = gnss_policy
    msg.has_gnss_age = gnss_age_s is not None
    msg.gnss_age_s = float(gnss_age_s or 0)
    msg.accepted_gnss_velocity_count = estimate.accepted_gnss_velocity_count
    msg.rejected_gnss_velocity_count = estimate.rejected_gnss_velocity_count
    msg.accepted_gnss_position_count = estimate.accepted_gnss_position_count
    msg.rejected_gnss_position_count = estimate.rejected_gnss_position_count
    msg.model_version, msg.compute_ms = estimate.model_version, compute_ms
    return msg
