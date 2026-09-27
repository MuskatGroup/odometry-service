import pytest
from odometry_core import (
    AdaptiveOdometryEstimator,
    AlongTrackPositionCorrection,
    ControlSample,
    DriveMap,
    InitialState,
    LongitudinalVelocityCorrection,
    ModelConfig,
    WheelSample,
)


def test_moving_start_bootstraps_once_without_disabling_subsequent_gates():
    estimator = AdaptiveOdometryEstimator(ModelConfig(bootstrap_wheel_speed=True))
    estimator.initialize(InitialState(0))
    estimator.ingest_control(ControlSample(0, 0, 0.6))
    estimator.ingest_wheel(WheelSample(20_000_000, 1, "front", 12.0))
    first = estimator.advance_to(20_000_000)
    assert first.v_mps == pytest.approx(12.0)
    assert first.accepted_wheel_count == 1
    estimator.ingest_wheel(WheelSample(40_000_000, 2, "front", 25.0))
    second = estimator.advance_to(40_000_000)
    assert second.v_mps < 13.0
    assert second.rejected_wheel_count == 1


def test_first_map_anchor_preserves_motion_history_and_future_events():
    estimator = AdaptiveOdometryEstimator(ModelConfig(c1=0.0))
    estimator.initialize(InitialState(0, 0.0, 10.0))
    estimator.ingest_control(ControlSample(0, 0, 0.2))
    estimator.ingest_wheel(WheelSample(100_000_000, 1, "front", 10.0))
    estimator.ingest_position_correction(AlongTrackPositionCorrection(
        200_000_000, 2, "route-a", 3000.0, 4.0, "gnss", initialize=True,
    ))
    estimator.ingest_wheel(WheelSample(300_000_000, 3, "front", 10.0))
    anchored = estimator.advance_to(200_000_000)
    assert anchored.s_m == pytest.approx(3000.0)
    assert anchored.v_mps == pytest.approx(10.0, abs=0.1)
    assert estimator.control == 0.2 and len(estimator.queue) == 1
    later = estimator.advance_to(300_000_000)
    assert later.accepted_wheel_count == 2
    assert later.accepted_gnss_position_count == 1
    assert 3000.9 < later.s_m < 3001.1
    # The same flag cannot repeatedly bypass the position innovation gate.
    estimator.ingest_position_correction(AlongTrackPositionCorrection(
        400_000_000, 4, "route-a", 9000.0, 4.0, "gnss", initialize=True,
    ))
    assert estimator.advance_to(400_000_000).rejected_gnss_position_count == 1


def test_adaptive_ekf_publishes_covariance_and_integrates_position():
    estimator = AdaptiveOdometryEstimator(ModelConfig(control_timeout_s=2.0))
    estimator.initialize(InitialState(0, 0, 10), ModelConfig(control_timeout_s=2.0))
    estimator.ingest_control(ControlSample(0, 0, 0, True))
    for index in range(1, 6):
        stamp = index * 1_000_000_000
        estimator.ingest_wheel(WheelSample(stamp, index, "left", 10, True))
        estimator.ingest_wheel(WheelSample(stamp, index, "right", 10, True))
    result = estimator.advance_to(5_000_000_000)
    assert result.model_version == "ekf-robust-v1"
    assert result.uncertainty_available
    assert len(result.covariance_4x4) == 16
    assert result.s_m == pytest.approx(48.9, abs=2.0)


def test_channel_vote_rejects_single_slipping_wheel():
    config = ModelConfig(control_timeout_s=2.0)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 5), config)
    estimator.ingest_control(ControlSample(0, 0, 0, True))
    estimator.ingest_wheel(WheelSample(100_000_000, 1, "good", 5, True))
    estimator.ingest_wheel(WheelSample(100_000_000, 1, "slip", 15, True))
    result = estimator.advance_to(100_000_000)
    assert result.v_mps < 6
    assert result.rejected_wheel_count >= 1


def test_model_only_horizon_and_covariance_growth():
    config = ModelConfig(control_timeout_s=None, max_model_only_s=2.0)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 2), config)
    estimator.ingest_control(ControlSample(0, 0, 0.2, True))
    estimator.ingest_wheel(WheelSample(0, 0, "left", 2, True))
    initial = estimator.advance_to(0)
    reserve = estimator.advance_to(1_000_000_000)
    expired = estimator.advance_to(3_000_000_000)
    assert reserve.mode == "MODEL_ONLY"
    assert reserve.sigma_s_m > initial.sigma_s_m
    assert expired.mode == "INVALID"
    assert "MODEL_ONLY_HORIZON" in expired.reason_codes


def test_frozen_channel_is_removed_while_acceleration_is_expected():
    config = ModelConfig(control_timeout_s=None, freeze_s=0.2, freeze_min_accel=0.05)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    for index in range(6):
        stamp = index * 100_000_000
        estimator.ingest_control(ControlSample(stamp, index, 0.8, True))
        estimator.ingest_wheel(WheelSample(stamp, index, "frozen", 1.0, True))
        estimator.ingest_wheel(WheelSample(stamp, index, "moving", 1.0 + 0.1 * index, True))
        result = estimator.advance_to(stamp)
    assert "WHEEL_FROZEN" in result.reason_codes
    assert "frozen" not in result.wheel_speeds_mps
    assert result.wheel_health["frozen"] == "frozen"


def test_common_freeze_remains_rejected_after_controller_returns_to_neutral():
    config = ModelConfig(c1=0.0, freeze_s=0.2, reacquire_s=1.0, max_model_only_s=1.0)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 5))
    for index in range(31):
        stamp = index * 100_000_000
        estimator.ingest_control(ControlSample(stamp, index, 0.8 if index < 6 else 0.0))
        for channel in ("front", "rear"):
            estimator.ingest_wheel(WheelSample(stamp, index, channel, 5.0))
        result = estimator.advance_to(stamp)
        if index == 5:
            accepted_before_coast = result.accepted_wheel_count
    assert set(result.wheel_health.values()) == {"frozen"}
    assert result.accepted_wheel_count == accepted_before_coast
    assert not result.valid
    assert "MODEL_ONLY_HORIZON" in result.reason_codes
    assert result.wheel_speeds_mps == {}

    # Fresh, physically consistent changes may recover, subject to normal hysteresis.
    for index in range(31, 41):
        stamp = index * 100_000_000
        for channel in ("front", "rear"):
            estimator.ingest_wheel(WheelSample(stamp, index, channel, estimator.x[1] + 0.001))
        result = estimator.advance_to(stamp)
    assert result.valid
    assert set(result.wheel_health.values()) == {"normal"}
    assert not estimator.frozen_ids


def test_stationary_braked_wheels_are_not_frozen_by_negative_model_acceleration():
    config = ModelConfig(
        freeze_s=0.2,
        braking_map=DriveMap((-1.0,), (0.0,), ((-0.8,),)),
    )
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0))
    for index in range(21):
        stamp = index * 100_000_000
        estimator.ingest_control(ControlSample(stamp, index, -1.0))
        for channel in ("front", "rear"):
            estimator.ingest_wheel(WheelSample(stamp, index, channel, 0.0))
        result = estimator.advance_to(stamp)
    assert result.valid
    assert result.v_mps == 0.0
    assert set(result.wheel_health.values()) == {"normal"}
    assert "WHEEL_FROZEN" not in result.reason_codes


def test_zero_speed_latch_can_recover_only_when_model_also_reaches_standstill():
    config = ModelConfig(
        c1=0.0, freeze_s=0.2, max_model_only_s=1.0,
        braking_map=DriveMap((-1.0,), (0.0,), ((-0.8,),)),
    )
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0))
    detected = False
    for index in range(41):
        stamp = index * 100_000_000
        estimator.ingest_control(ControlSample(stamp, index, 1.0 if index < 10 else -1.0))
        for channel in ("front", "rear"):
            estimator.ingest_wheel(WheelSample(stamp, index, channel, 0.0))
        result = estimator.advance_to(stamp)
        detected |= "WHEEL_FROZEN" in result.reason_codes
        if index == 10:
            assert result.v_mps > 0.0
            assert set(result.wheel_health.values()) == {"frozen"}
    assert detected
    assert result.valid and result.mode == "FUSED"
    assert result.v_mps == 0.0
    assert set(result.wheel_health.values()) == {"normal"}
    assert not estimator.frozen_ids


def test_scalar_velocity_and_position_corrections_are_ordered_and_counted():
    config = ModelConfig(control_timeout_s=None, position_correction_gate=1_000)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    stamp = 1_000_000_000
    estimator.ingest_position_correction(
        AlongTrackPositionCorrection(stamp, 4, "route-a", 4.0, 0.1, "gnss")
    )
    estimator.ingest_velocity_correction(
        LongitudinalVelocityCorrection(stamp, 3, 2.0, 0.1, "gnss")
    )
    estimator.ingest_control(ControlSample(stamp, 1, 0.0))
    estimator.ingest_wheel(WheelSample(stamp, 2, "front", 2.0))
    result = estimator.advance_to(stamp)
    assert result.route_id == "route-a"
    assert result.accepted_gnss_velocity_count == 1
    assert result.accepted_gnss_position_count == 1
    assert result.s_m > 1.0


def test_wheel_fusion_runs_before_gnss_corrections_in_the_same_group(monkeypatch):
    # Regression for organizer audit 2026-09-27: events of one timestamp are queued
    # control -> wheels -> velocity correction -> position correction (types.event_key), but
    # _process_group used to apply each correction the moment it was seen while deferring the
    # wheel fusion to after its whole loop -- so a correction in the same group as a wheel sample
    # was actually applied *before* that wheel update, the reverse of the documented order.
    config = ModelConfig(control_timeout_s=None, position_correction_gate=1_000)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    stamp = 1_000_000_000
    calls = []
    monkeypatch.setattr(
        estimator,
        "_wheel_update",
        lambda *a, **k: (calls.append("wheel"), AdaptiveOdometryEstimator._wheel_update(estimator, *a, **k))[-1],
    )
    monkeypatch.setattr(
        estimator,
        "_correction_update",
        lambda event: (
            calls.append("velocity" if isinstance(event, LongitudinalVelocityCorrection) else "position"),
            AdaptiveOdometryEstimator._correction_update(estimator, event),
        )[-1],
    )
    estimator.ingest_control(ControlSample(stamp, 1, 0.0))
    estimator.ingest_wheel(WheelSample(stamp, 2, "front", 2.0))
    estimator.ingest_velocity_correction(LongitudinalVelocityCorrection(stamp, 3, 2.0, 0.1, "gnss"))
    estimator.ingest_position_correction(
        AlongTrackPositionCorrection(stamp, 4, "route-a", 4.0, 0.1, "gnss")
    )
    estimator.advance_to(stamp)
    assert calls == ["wheel", "velocity", "position"]


def test_gnss_correction_alone_in_a_group_still_applies():
    # The early return that used to skip straight past any pending corrections when a group had
    # no wheel samples at all (organizer audit, 2026-09-27) would have made this a no-op.
    config = ModelConfig(control_timeout_s=None, velocity_correction_gate=1_000)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    stamp = 1_000_000_000
    estimator.ingest_velocity_correction(LongitudinalVelocityCorrection(stamp, 1, 5.0, 0.01, "gnss"))
    result = estimator.advance_to(stamp)
    assert result.accepted_gnss_velocity_count == 1
    assert result.v_mps == pytest.approx(5.0, abs=0.5)


def test_wrong_route_and_large_gnss_outlier_are_rejected():
    config = ModelConfig(control_timeout_s=None)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    estimator.ingest_position_correction(
        AlongTrackPositionCorrection(0, 1, "route-a", 0.1, 1.0, "gnss")
    )
    estimator.advance_to(0)
    estimator.ingest_position_correction(
        AlongTrackPositionCorrection(1, 2, "route-b", 0.2, 1.0, "gnss")
    )
    estimator.ingest_velocity_correction(
        LongitudinalVelocityCorrection(1, 3, 100.0, 0.01, "gnss")
    )
    result = estimator.advance_to(1)
    assert result.rejected_gnss_position_count == 1
    assert result.rejected_gnss_velocity_count == 1


def test_rejected_channels_do_not_keep_filter_in_fused_mode():
    config = ModelConfig(control_timeout_s=None)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 5), config)
    estimator.ingest_control(ControlSample(0, 0, 1.0))
    estimator.ingest_wheel(WheelSample(100_000_000, 1, "front", 20.0))
    estimator.ingest_wheel(WheelSample(100_000_000, 1, "rear", 20.0))
    result = estimator.advance_to(100_000_000)
    assert result.wheel_speeds_mps == {}
    assert result.mode == "MODEL_ONLY"
    assert set(result.wheel_health.values()) == {"positive_slip"}


def test_gap_reset_preserves_route_lock():
    config = ModelConfig(control_timeout_s=None, max_gap_s=1.0)
    estimator = AdaptiveOdometryEstimator(config)
    estimator.initialize(InitialState(0, 0, 1), config)
    estimator.route_id = "route-a"
    assert estimator.advance_to(2_000_000_000).route_id == "route-a"


def test_drive_map_interpolation_and_grade_change_prediction():
    drive = DriveMap(
        (0.5, 1.0),
        (0.0, 10.0),
        ((0.5, 0.25), (1.0, 0.5)),
    )
    assert drive.evaluate(0.75, 5.0)[0] == pytest.approx(0.5625)
    config = ModelConfig(control_timeout_s=None, traction_map=drive, c1=0.0)
    flat = AdaptiveOdometryEstimator(config, grade_provider=lambda _s: 0.0)
    uphill = AdaptiveOdometryEstimator(config, grade_provider=lambda _s: 0.02)
    for estimator in (flat, uphill):
        estimator.initialize(InitialState(0, 0, 5), config)
        estimator.ingest_control(ControlSample(0, 0, 0.75))
    assert uphill.advance_to(1_000_000_000).v_mps < flat.advance_to(1_000_000_000).v_mps
