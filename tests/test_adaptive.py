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
from odometry_io import probe
from odometry_lab.evaluate import evaluate
from odometry_lab.storage import write_rows


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


def test_probe_writes_review_required_draft(tmp_path):
    source = tmp_path / "sample.csv"
    source.write_text("timestamp_ns,handle,wheel_left_mps,wheel_right_mps\n0,0,1,1\n", encoding="utf-8")
    result = probe(source)
    assert result["profile"]["layout"] == "wide"
    assert len(result["profile"]["wheels"]) == 2
    assert any("REVIEW_REQUIRED" in warning for warning in result["warnings"])


def test_extended_metrics_and_fault_drift(tmp_path):
    estimates = tmp_path / "estimates.jsonl"
    truth = tmp_path / "truth.jsonl"
    faults = tmp_path / "faults.jsonl"
    write_rows(
        estimates,
        [
            {
                "stamp_ns": "0",
                "v_mps": 1.0,
                "s_m": 0.0,
                "valid": True,
                "mode": "FUSED",
                "reason_codes": [],
                "sigma_v_mps": 0.2,
            },
            {
                "stamp_ns": "1000000000",
                "v_mps": 2.0,
                "s_m": 1.5,
                "valid": True,
                "mode": "DEGRADED",
                "reason_codes": ["WHEEL_REJECTED"],
                "sigma_v_mps": 0.2,
            },
        ],
    )
    write_rows(
        truth,
        [
            {"stamp_ns": "0", "v_mps": 1.0, "s_m": 0.0},
            {"stamp_ns": "1000000000", "v_mps": 1.0, "s_m": 1.0},
        ],
    )
    write_rows(faults, [{"type": "slip", "start_ns": "0", "end_ns": "1000000000"}])
    report = evaluate(estimates, truth, faults_path=faults)
    assert report["metrics"]["speed_max_abs_mps"] == 1.0
    assert report["metrics"]["mode_fraction"]["DEGRADED"] == 0.5
    assert report["fault_windows"][0]["position_drift_m"] == 0.5
