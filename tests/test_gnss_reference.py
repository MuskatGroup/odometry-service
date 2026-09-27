import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python/odometry_geometry/src"))
sys.path.insert(0, str(ROOT / "dataset/analysis"))

import build_reference_table as brt  # noqa: E402
import gnss_reference as gr  # noqa: E402
from odometry_geometry import PathGraph, ProjectionConfig, TramGeometry, project_wgs84  # noqa: E402

NS = 1_000_000_000
CONFIG = ProjectionConfig()
ORIGIN = (55.8088325462547, 37.4602768500852)  # organizer control point, x=103501.63 y=85876.12


def inverse(x_m: float, y_m: float) -> tuple[float, float]:
    """WGS84 lat/lon of a map point (Newton on the forward projection)."""
    lat, lon = ORIGIN
    for _ in range(8):
        p = project_wgs84(lat, lon, 0.0, CONFIG)
        h = 1e-6
        px = project_wgs84(lat, lon + h, 0.0, CONFIG)
        py = project_wgs84(lat + h, lon, 0.0, CONFIG)
        a, b = (px.x_m - p.x_m) / h, (py.x_m - p.x_m) / h
        c, d = (px.y_m - p.y_m) / h, (py.y_m - p.y_m) / h
        ex, ey = x_m - p.x_m, y_m - p.y_m
        det = a * d - b * c
        lon += (d * ex - b * ey) / det
        lat += (-c * ex + a * ey) / det
    return lat, lon


def straight_graph():
    x0, y0 = 103400.0, 85876.0
    return PathGraph(
        {"east": [{"x": x0 + i, "y": y0, "z": 170.0, "tang": 0.0, "curv": 0.0} for i in range(400)]}
    ), x0, y0


def fixes_for_run(x0, y0, speed_mps, seconds=20.0, base_start_s=30.0):
    geometry = TramGeometry()
    out = []
    for k in range(int(seconds * 10)):
        t = k * 0.1
        base_x = x0 + base_start_s + speed_mps * t
        for receiver, antenna, offset in (
            ("master", geometry.antenna_1_xyz_m, 0.0),
            ("rover", geometry.antenna_2_xyz_m, 0.03),
        ):
            lat, lon = inverse(base_x + antenna[0] + speed_mps * offset, y0)
            out.append(gr.GnssFix(round((t + offset) * NS), receiver, lat, lon, 170.0 + antenna[2], 2))
    return out


def velocities(speed_master, speed_rover, seconds=20.0):
    out = []
    for k in range(int(seconds * 10)):
        t = k * 0.1
        if speed_master is not None:
            out.append(gr.GnssVelocity(round(t * NS), "master", speed_master(t), 0.0, 0.0))
        if speed_rover is not None:
            out.append(gr.GnssVelocity(round((t + 0.03) * NS), "rover", 0.0, speed_rover(t), 0.0))
    return out


def test_consensus_of_two_sources_uses_horizontal_magnitude():
    graph, *_ = straight_graph()
    samples = gr.SpeedReferenceBuilder(graph).build([], velocities(lambda t: 5.0, lambda t: 5.1))
    assert samples[10].reason == gr.OK
    assert samples[10].v_ref_mps == pytest.approx(5.05, abs=0.02)
    assert samples[10].uncertainty_mps == pytest.approx(0.05, abs=0.02)
    assert samples[10].s_ref_m is None  # no fixes: unavailable, not zero


def test_single_source_and_disagreement_and_empty():
    graph, *_ = straight_graph()
    builder = gr.SpeedReferenceBuilder(graph)
    single = builder.build([], velocities(lambda t: 4.0, None))
    assert single[5].reason == gr.SINGLE_SOURCE and single[5].v_ref_mps == pytest.approx(4.0)
    split = builder.build([], velocities(lambda t: 4.0, lambda t: 6.0))
    assert split[10].reason == gr.SOURCES_DISAGREE and split[10].v_ref_mps is None
    assert builder.build([], []) == []
    assert gr.coverage(split) < 0.1 and gr.coverage([]) == 0.0


def test_position_and_speed_agree_on_a_straight_route():
    graph, x0, y0 = straight_graph()
    speed = 5.0
    samples = gr.SpeedReferenceBuilder(graph).build(
        fixes_for_run(x0, y0, speed), velocities(lambda t: speed, lambda t: speed)
    )
    middle = samples[100]
    assert middle.route_id == "east"
    expected_s = 30.0 + speed * middle.stamp_ns / NS
    assert middle.s_ref_m == pytest.approx(expected_s, abs=0.1)  # base_link, not an antenna
    assert middle.v_ref_mps == pytest.approx(speed, abs=1e-6)
    assert all(s.reason != gr.POSITION_MISMATCH for s in samples[20:-20])


def test_speed_that_contradicts_position_is_flagged():
    graph, x0, y0 = straight_graph()
    samples = gr.SpeedReferenceBuilder(graph).build(
        fixes_for_run(x0, y0, 5.0), velocities(lambda t: 8.0, lambda t: 8.0)
    )
    assert samples[100].reason == gr.POSITION_MISMATCH
    assert samples[100].uncertainty_mps >= 1.0


def test_fix_without_status_is_ignored():
    graph, x0, y0 = straight_graph()
    bad = [gr.GnssFix(f.stamp_ns, f.receiver, f.latitude_deg, f.longitude_deg, f.altitude_m, -1)
           for f in fixes_for_run(x0, y0, 5.0)]
    samples = gr.SpeedReferenceBuilder(graph).build(bad, velocities(lambda t: 5.0, lambda t: 5.0))
    assert all(s.s_ref_m is None for s in samples)


def test_acceleration_from_speed_ramp():
    graph, *_ = straight_graph()
    samples = gr.SpeedReferenceBuilder(graph).build(
        [], velocities(lambda t: 0.5 * t, lambda t: 0.5 * t)
    )
    accelerations = gr.acceleration(samples)
    assert accelerations[50] == pytest.approx(0.5, abs=0.02)
    assert accelerations[0] is None  # no look-back at the edge


def test_acceleration_is_none_where_reference_is_missing():
    graph, *_ = straight_graph()
    values = velocities(lambda t: 3.0, lambda t: 3.0)
    values = [v for v in values if not 8 * NS < v.stamp_ns < 12 * NS]  # 4 s dropout
    samples = gr.SpeedReferenceBuilder(graph).build([], values)
    accelerations = gr.acceleration(samples)
    gap = [i for i, s in enumerate(samples) if s.v_ref_mps is None]
    assert all(accelerations[i] is None for i in gap)


def test_splits_keep_whole_bags_and_duplicates_together():
    groups = {f"g{i}": [f"30618_{i}a", f"30618_{i}b"] if i % 3 == 0 else [f"30618_{i}a"] for i in range(20)}
    usable = {bag for bags in groups.values() for bag in bags} - {"30618_1a"}
    split = brt.assign_splits(groups, usable)
    named = ("identification", "validation", "test")
    everything = [bag for name in named for bag in split[name]]
    assert len(everything) == len(set(everything)) == len(usable)
    assert split["unusable"] == ["30618_1a"]
    for bags in groups.values():
        homes = {name for name in named for bag in bags if bag in split[name]}
        assert len(homes) <= 1  # duplicates never straddle two splits
    assert split == brt.assign_splits(dict(reversed(list(groups.items()))), usable)  # order-independent
    assert split["identification"] and split["validation"] and split["test"]


def test_reference_values_are_finite_numbers():
    graph, x0, y0 = straight_graph()
    samples = gr.SpeedReferenceBuilder(graph).build(
        fixes_for_run(x0, y0, 5.0), velocities(lambda t: 5.0, lambda t: 5.0)
    )
    for s in samples:
        for value in (s.v_ref_mps, s.s_ref_m, s.uncertainty_mps):
            assert value is None or math.isfinite(value)
