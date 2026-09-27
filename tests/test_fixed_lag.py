import importlib.util
from pathlib import Path

import pytest
from odometry_core import AdaptiveOdometryEstimator, ControlSample, InitialState, ModelConfig, WheelSample

MODULE = (
    Path(__file__).parents[1]
    / "ros2_ws"
    / "src"
    / "odometry_node"
    / "odometry_node"
    / "fixed_lag.py"
)
SPEC = importlib.util.spec_from_file_location("fixed_lag", MODULE)
fixed_lag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fixed_lag)

NS = 1_000_000_000


def test_delayed_wheels_are_committed_in_order_while_output_is_current():
    estimator = AdaptiveOdometryEstimator(ModelConfig(control_timeout_s=None))
    estimator.initialize(InitialState(0, 0.0, 10.0))
    estimator.ingest_control(ControlSample(0, 0, 0.0))
    estimator.ingest_wheel(WheelSample(0, 1, "front", 10.0))
    estimator.ingest_wheel(WheelSample(0, 2, "rear", 10.0))

    first, first_commit = fixed_lag.advance_fixed_lag(estimator, 100_000_000, 120_000_000)
    assert first.stamp_ns == 100_000_000
    assert first_commit == estimator.t == 0

    # This sample arrives at processing time 150 ms: its header is 100 ms late, but the
    # committed filter is still at 0 and can accept it without rewriting the timestamp.
    estimator.ingest_wheel(WheelSample(50_000_000, 3, "front", 10.0))
    estimator.ingest_wheel(WheelSample(50_000_000, 4, "rear", 10.0))
    estimator.ingest_wheel(WheelSample(80_000_000, 5, "front", 10.0))
    estimator.ingest_wheel(WheelSample(80_000_000, 6, "rear", 10.0))
    current, commit = fixed_lag.advance_fixed_lag(estimator, 170_000_000, 120_000_000)

    assert commit == estimator.t == 50_000_000
    assert current.stamp_ns == 170_000_000
    assert estimator.rejected == 0
    # Same-timestamp wheel pairs produce one median update. The preview sees the 80 ms
    # group, while the committed estimator keeps it queued for the next watermark.
    assert current.accepted_wheel_count == 3
    assert estimator.accepted == 2
    assert len(estimator.queue) == 2
    assert "OUT_OF_ORDER" not in current.reason_codes


def test_event_older_than_committed_watermark_is_still_rejected():
    estimator = AdaptiveOdometryEstimator(ModelConfig(control_timeout_s=None))
    estimator.initialize(InitialState(0))
    fixed_lag.advance_fixed_lag(estimator, 200_000_000, 100_000_000)
    estimator.ingest_wheel(WheelSample(99_000_000, 1, "front", 1.0))
    result, _ = fixed_lag.advance_fixed_lag(estimator, 220_000_000, 100_000_000)
    assert estimator.rejected == 1
    assert "OUT_OF_ORDER" in result.reason_codes


def test_freeze_latch_is_previewed_without_mutating_committed_health():
    estimator = AdaptiveOdometryEstimator(ModelConfig(c1=0.0, freeze_s=0.2))
    estimator.initialize(InitialState(0, 0.0, 5.0))
    for index in range(7):
        stamp = index * 100_000_000
        estimator.ingest_control(ControlSample(stamp, index, 0.8 if index < 3 else 0.0))
        for channel in ("front", "rear"):
            estimator.ingest_wheel(WheelSample(stamp, index, channel, 5.0))
    preview, commit = fixed_lag.advance_fixed_lag(estimator, 350_000_000, 300_000_000)
    assert commit == 50_000_000
    assert not estimator.frozen_ids
    assert set(preview.wheel_health.values()) == {"frozen"}
    preview, commit = fixed_lag.advance_fixed_lag(estimator, 600_000_000, 300_000_000)
    assert commit == 300_000_000
    assert estimator.frozen_ids == {"front", "rear"}
    assert preview.wheel_speeds_mps == {}
    assert set(preview.wheel_health.values()) == {"frozen"}


def test_route_grade_provider_is_not_copied_with_its_owner_lock():
    import threading

    class Grade:
        def __init__(self):
            self.lock = threading.Lock()

        def at(self, route, s):
            return 0.01

    source = Grade()
    estimator = AdaptiveOdometryEstimator(route_grade_provider=source.at)
    estimator.initialize(InitialState(0, 0.0, 10.0))
    estimator.route_id = "route"
    current, _ = fixed_lag.advance_fixed_lag(estimator, 100_000_000, 120_000_000)
    assert current.v_mps < 10.0


@pytest.mark.parametrize("now, lag", [(-1, 0), (0, -1)])
def test_invalid_fixed_lag_arguments(now, lag):
    estimator = AdaptiveOdometryEstimator()
    estimator.initialize(InitialState(0))
    with pytest.raises(ValueError):
        fixed_lag.advance_fixed_lag(estimator, now, lag)
