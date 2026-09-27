from pathlib import Path

from odometry_io import load_profile
from odometry_lab.evaluate import evaluate
from odometry_lab.runner import run
from odometry_lab.scenario import generate
from odometry_lab.storage import read_rows, write_rows

ROOT = Path(__file__).resolve().parents[1]


def test_generator_repeatable_and_truth_is_separate(tmp_path):
    a, b = tmp_path / "a", tmp_path / "b"
    generate(a)
    generate(b)
    assert (a / "events.jsonl").read_bytes() == (b / "events.jsonl").read_bytes()
    assert all("s_m" not in row and "fault" not in row for row in read_rows(a / "events.jsonl"))
    rows = read_rows(a / "events.jsonl")
    assert not any(r["kind"] == "wheel" and 8e9 <= int(r["stamp_ns"]) < 11e9 for r in rows)


def test_full_offline_demo_and_no_truth_metric(tmp_path):
    generate(tmp_path)
    run(
        str(tmp_path / "events.jsonl"),
        load_profile(ROOT / "contracts/profiles/events.yaml"),
        tmp_path,
        "test-run",
    )
    report = evaluate(tmp_path / "estimates.jsonl", tmp_path / "truth.jsonl")
    assert report["metrics"]["sample_count"] == 1001
    assert report["metrics"]["speed_rmse_mps"] >= 0
    assert 0 < report["metrics"]["valid_fraction"] < 1
    assert evaluate(tmp_path / "estimates.jsonl")["metrics"]["speed_rmse_mps"] is None


def test_exact_estimate_and_interpolation(tmp_path):
    rows = [
        {
            "stamp_ns": str(i * 1_000_000_000),
            "v_mps": 10.0,
            "s_m": i * 10.0,
            "valid": True,
            "mode": "FUSED",
            "reason_codes": [],
        }
        for i in range(6)
    ]
    write_rows(tmp_path / "estimate.jsonl", rows)
    write_rows(tmp_path / "truth.jsonl", [rows[0], rows[-1]])
    report = evaluate(tmp_path / "estimate.jsonl", tmp_path / "truth.jsonl")
    assert report["metrics"]["speed_rmse_mps"] == 0
    assert report["metrics"]["position_mae_m"] == 0
    assert report["metrics"]["valid_fraction"] == 1


# --- real-bag lab (odometry_lab.realdata / calibrate) ----------------------------------------------

import pytest  # noqa: E402
from odometry_core import (  # noqa: E402
    AlongTrackPositionCorrection,
    ControlSample,
    InitialState,
    LongitudinalVelocityCorrection,
    WheelSample,
)
from odometry_lab import realdata as rd  # noqa: E402
from odometry_lab.calibrate import config_from_profile  # noqa: E402

NS = 1_000_000_000


def wheel_stream(seconds=10, hz=10):
    out = []
    for k in range(seconds * hz):
        for wheel in ("front", "rear"):
            out.append(WheelSample(k * NS // hz, len(out), wheel, 5.0 + 0.01 * k))
    return out


def test_inject_faults_only_touch_selected_channel_and_window():
    events = wheel_stream()
    start, end = 3 * NS, 6 * NS
    dropped = rd.inject(events, "dropout", ("front",), start, end)
    assert not [e for e in dropped if e.wheel_id == "front" and start <= e.stamp_ns < end]
    assert len([e for e in dropped if e.wheel_id == "rear"]) == len([e for e in events if e.wheel_id == "rear"])
    frozen = rd.inject(events, "freeze", ("front",), start, end)
    values = {e.speed_mps for e in frozen if e.wheel_id == "front" and start <= e.stamp_ns < end}
    assert len(values) == 1
    locked = rd.inject(events, "lock", ("rear",), start, end)
    assert {e.speed_mps for e in locked if e.wheel_id == "rear" and start <= e.stamp_ns < end} == {0.0}
    slipped = rd.inject(events, "slip", ("front",), start, end)
    before = next(e for e in events if e.wheel_id == "front" and e.stamp_ns == 4 * NS)
    after = next(e for e in slipped if e.wheel_id == "front" and e.stamp_ns == 4 * NS)
    assert after.speed_mps == pytest.approx(before.speed_mps * 1.5)
    assert rd.inject(events, "none", (), start, end) == events
    with pytest.raises(ValueError):
        rd.inject(events, "explode", ("front",), start, end)


def test_inject_leaves_control_events_alone():
    events = [*wheel_stream(2), ControlSample(5 * NS // 10, 999, 0.5)]
    out = rd.inject(events, "dropout", ("front", "rear"), 0, 10 * NS)
    assert [e for e in out if isinstance(e, ControlSample)] == [events[-1]]


def test_choose_window_needs_reference_and_prefers_motion():
    rows = [{"stamp_ns": k * NS // 10, "v": 0.0, "s": 0.0} for k in range(1500)]  # 150 s standing
    rows += [{"stamp_ns": (1500 + k) * NS // 10, "v": 0.01 * k, "s": 0.5 * k} for k in range(1500)]
    window = rd.choose_window(rows, 100.0)
    assert window is not None and window.start_ns >= 150 * NS
    assert rd.choose_window(rows[:200], 100.0) is None
    holes = [{**r, "s": None} for r in rows]
    assert rd.choose_window(holes, 100.0) is None  # no position -> no window


def test_gnss_masks():
    stamps = [k * NS // 10 for k in range(1300)]
    assert not any(rd.gnss_availability(stamps, "never"))
    initial = rd.gnss_availability(stamps, "initial")
    assert initial[50] and not initial[200]
    burst = rd.gnss_availability(stamps, "intermittent-60")
    assert burst[610] and not burst[700] and not burst[300]
    with pytest.raises(ValueError):
        rd.gnss_availability(stamps, "sometimes")
    assert rd.gnss_availability([], "never") == []


def test_gnss_reference_is_converted_to_ordered_corrections_and_ingested():
    stamps = [k * NS // 10 for k in range(30)]
    events = []
    for k, stamp in enumerate(stamps):
        events.extend(
            [
                ControlSample(stamp, 3 * k, 0.0),
                WheelSample(stamp, 3 * k + 1, "front", 5.0),
                WheelSample(stamp, 3 * k + 2, "rear", 5.0),
            ]
        )
    truth = [
        {
            "stamp_ns": str(stamp),
            "v_mps": 5.0,
            "s_m": 5.0 * stamp / NS,
            "route_id": "route-a",
            "velocity_sigma_mps": 0.2,
        }
        for stamp in stamps
    ]
    case = rd.RealCase(
        bag_id="synthetic",
        vehicle_id="test",
        window=rd.Window(stamps[0], stamps[-1]),
        events=events,
        truth=truth,
        initial=InitialState(stamps[0], 0.0, 5.0),
        reference_coverage=1.0,
        route_id="route-a",
    )

    corrections = rd.correction_events(case, "initial")
    assert len(corrections) == 2 * len(stamps)
    assert isinstance(corrections[0], LongitudinalVelocityCorrection)
    assert isinstance(corrections[1], AlongTrackPositionCorrection)
    result = rd.run_case(case, "none", "M1", fault_start_s=10.0, gnss_mode="initial")
    assert result["counts"]["accepted_gnss_velocity"] > 0
    assert result["counts"]["accepted_gnss_position"] > 0
    assert result["correction_jump_m"] is not None
    with pytest.raises(ValueError, match="adaptive-ekf"):
        rd.run_case(case, "none", "B0", gnss_mode="initial")


def test_recovery_time_and_unavailable_when_never_recovered():
    truth = [{"stamp_ns": str(k * NS // 10), "v_mps": 5.0, "s_m": 0.0} for k in range(200)]

    def frames(recover_at):
        return [
            {"stamp_ns": str(k * NS // 10), "v_mps": 5.0 if k >= recover_at else 9.0, "valid": True}
            for k in range(200)
        ]

    assert rd.recovery_time_s(frames(120), truth, 100 * NS // 10) == pytest.approx(2.0)
    assert rd.recovery_time_s(frames(10_000), truth, 100 * NS // 10) is None


def test_dedupe_keeps_first_of_each_duplicate_group():
    bags = ["a", "b", "c", "d"]
    assert rd.dedupe(bags, [["b", "a"], ["d", "c"]]) == ["a", "c"]


def test_summary_keeps_missing_metrics_unavailable():
    rows = [
        {"scenario": "s", "preset": "p", "speed_rmse_mps": 1.0, "speed_bias_mps": None,
         "speed_p95_abs_mps": 2.0, "position_rmse_m": 3.0, "final_position_error_m": 4.0,
         "availability": 1.0, "model_only_fraction": 0.0, "recovery_s": None},
        {"scenario": "s", "preset": "p", "speed_rmse_mps": 3.0, "speed_bias_mps": None,
         "speed_p95_abs_mps": 2.0, "position_rmse_m": 3.0, "final_position_error_m": 4.0,
         "availability": 1.0, "model_only_fraction": 0.0, "recovery_s": None},
    ]  # fmt: skip
    (entry,) = rd.summarize(rows)
    assert entry["speed_rmse_mps"] == 2.0 and entry["bags"] == 2
    assert entry["speed_bias_mps"] is None and entry["recovery_s"] is None


def test_model_profile_maps_to_estimator_config():
    profile = {
        "longitudinal": {"tau_s": 0.4, "c1_inv_s": 0.02, "c2_inv_m": 0.001},
        "noise": {"q_v": 0.05, "q_a": 3.0, "q_d": 0.002, "wheel_variance_floor": 0.05},
        "wheel_health": {
            "gate_normal": 9.0, "gate_reject": 36.0, "max_wheel_accel_mps2": 6.0,
            "freeze_s": 1.0, "reacquire_s": 10.0, "recover_updates": 5,
        },
    }  # fmt: skip
    config = config_from_profile(profile)
    assert (config.tau_s, config.c1, config.q_a, config.gate_reject) == (0.4, 0.02, 3.0, 36.0)
    with pytest.raises(KeyError):
        config_from_profile({**profile, "noise": {}})


def test_real_bag_case_and_faulted_run_when_dataset_is_available():
    bag = ROOT / "dataset/data/30618_2050d396"
    reference = ROOT / "dataset/derived/reference/30618_2050d396.csv"
    if not (bag.exists() and reference.exists()):
        pytest.skip("real bag or derived reference not available")
    case = rd.build_case(bag, reference, window_s=60.0)
    assert case is not None and case.initial.v_mps is not None
    assert all(isinstance(e.speed_mps, float) for e in case.events if isinstance(e, WheelSample))
    clean = rd.run_case(case, "none", "B0", fault_start_s=20.0, fault_len_s=10.0)
    faulted = rd.run_case(case, "lock-1", "B0", fault_start_s=20.0, fault_len_s=10.0)
    assert clean["speed_rmse_mps"] < 0.5 < faulted["speed_rmse_mps"]
    assert clean["recovery_s"] is None and faulted["correction_jump_m"] is None


def test_export_run_writes_console_files_with_map_and_null_corrections(tmp_path):
    from odometry_lab.exportrun import export_run

    bag = ROOT / "dataset/data/30618_2050d396"
    reference = ROOT / "dataset/derived/reference/30618_2050d396.csv"
    if not (bag.exists() and reference.exists()):
        pytest.skip("real bag or derived reference not available")
    case = rd.build_case(bag, reference, window_s=40.0)
    report = export_run(case, "lock-1", "B0", tmp_path, "test-run", fault_start_s=10.0, fault_len_s=10.0)
    assert report["schema_version"] == "0.3"
    assert report["scenario"]["fault"] == "lock" and report["fault_windows"]
    assert report["corrections"]["available"] is False
    track = report["map"]["estimate"]
    assert track and all(0 < p["x_m"] < 200_000 and 0 < p["y_m"] < 200_000 for p in track)
    assert {row["route_id"] for row in report["map"]["routes"]} >= {report["map"]["route_id"]}
    frames = read_rows(tmp_path / "estimates.jsonl")
    assert frames[0]["run_id"] == "test-run" and frames[0]["schema_version"] == "0.3"
    assert (tmp_path / "run.json").exists()
