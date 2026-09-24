import pytest
from odometry_core import ControlSample, EstimatorConfig, InitialState, OdometryEstimator, WheelSample


def create(**kwargs):
    model = OdometryEstimator()
    model.initialize(InitialState(0, 0, 10), EstimatorConfig(**kwargs))
    return model


def test_constant_speed_integrates_fifty_meters_and_no_false_certainty():
    model = create()
    for index in range(251):
        t = index * 20_000_000
        model.ingest_control(ControlSample(t, index, 0))
        model.ingest_wheel(WheelSample(t, index, "a", 10))
        result = model.advance_to(t)
    assert result.s_m == pytest.approx(50)
    assert result.valid
    assert result.uncertainty_available is False
    assert result.sigma_v_mps is None
    assert result.covariance_4x4 is None


def test_median_duplicate_late_and_invalid():
    model = create()
    model.ingest_control(ControlSample(0, 1, 0))
    for channel, velocity in [("a", 8), ("b", 10), ("c", 100)]:
        sample = WheelSample(0, 1, channel, velocity)
        model.ingest_wheel(sample)
        model.ingest_wheel(sample)
    assert model.advance_to(0).v_mps == 10
    model.ingest_wheel(WheelSample(0, 2, "a", 1))
    model.ingest_control(ControlSample(1, 2, float("nan")))
    model.advance_to(1)
    assert model.counts["DUPLICATE"] == 3
    assert model.counts["OUT_OF_ORDER"] == 1
    assert model.counts["INVALID_INPUT"] == 1


def test_timeouts_and_no_automatic_zero_for_missing_wheels():
    model = create()
    model.ingest_control(ControlSample(0, 0, 0))
    model.ingest_wheel(WheelSample(0, 0, "a", 10))
    model.advance_to(0)
    model.ingest_control(ControlSample(600_000_000, 1, 0))
    result = model.advance_to(600_000_000)
    assert result.mode == "DEGRADED"
    assert result.v_mps == 10
    assert result.s_m == pytest.approx(6)
    result = model.advance_to(3_000_000_000)
    assert result.mode == "INVALID"
    assert not result.valid
    assert {"WHEEL_STALE", "CONTROL_STALE"} <= set(result.reason_codes)


def test_expiration_is_independent_of_publication_frequency():
    def advance(ticks):
        model = create()
        model.ingest_wheel(WheelSample(0, 0, "a", 10))
        model.ingest_wheel(WheelSample(200_000_000, 1, "b", 20))
        for t in ticks:
            result = model.advance_to(t)
        return result.s_m

    assert advance([1_000_000_000]) == pytest.approx(advance(list(range(0, 1_000_000_001, 10_000_000))))


def test_queue_and_channel_limits_are_visible():
    model = create(max_queue=1, max_channels=1)
    model.ingest_wheel(WheelSample(0, 0, "a", 10))
    model.ingest_wheel(WheelSample(0, 0, "b", 10))
    model.advance_to(0)
    model.ingest_wheel(WheelSample(1, 1, "b", 10))
    model.advance_to(1)
    assert model.counts["QUEUE_OVERFLOW"] == 1
    assert model.counts["CHANNEL_LIMIT"] == 1


def test_uninitialized_and_clock_reset():
    model = OdometryEstimator()
    assert model.advance_to(0).mode == "INITIALIZING"
    model.initialize(InitialState(0), EstimatorConfig())
    model.advance_to(10)
    with pytest.raises(ValueError):
        model.advance_to(9)
    model.reset()
    assert model.advance_to(0).s_m is None
