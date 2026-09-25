import pytest
from odometry_core import AdaptiveOdometryEstimator, ControlSample, InitialState, ModelConfig, WheelSample
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
