import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "python/odometry_geometry/src"))

from odometry_geometry import (  # noqa: E402
    STATUS_AMBIGUOUS,
    STATUS_MATCHED,
    STATUS_OUT_OF_GRAPH,
    MapPoint,
    OutOfGraphError,
    PathGraph,
    ProjectionConfig,
    TramGeometry,
    base_link_from_dual_antenna,
    base_link_from_single_antenna,
    project_wgs84,
)

PATHGRAPH = ROOT / "dataset/Pathgraph"


def straight_route(yaw=0.0, x0=0.0, y0=0.0, n=101, dz=0.0):
    return [
        {
            "x": x0 + i * math.cos(yaw),
            "y": y0 + i * math.sin(yaw),
            "z": i * dz,
            "tang": yaw,
            "curv": 0.0,
        }
        for i in range(n)
    ]


def two_track_graph():
    # forward track along +x at y=0, neighbouring opposite track along -x at y=3.7
    return PathGraph(
        {"fwd": straight_route(0.0), "back": straight_route(math.pi, x0=100.0, y0=3.7)}
    )


# --- projection -----------------------------------------------------------------------------


def test_organizer_control_point():
    p = project_wgs84(55.8088325462547, 37.4602768500852, 0.0, ProjectionConfig())
    assert p.x_m == pytest.approx(103501.6309, abs=0.02)
    assert p.y_m == pytest.approx(85876.1201, abs=0.02)


def test_no_modulo_jump_across_100km_boundary():
    config = ProjectionConfig()
    xs = [project_wgs84(55.8, lon, 0.0, config).x_m for lon in (37.30, 37.32, 37.34, 37.36)]
    assert all(b > a for a, b in zip(xs, xs[1:], strict=False))
    assert all(x > 90000 for x in xs)  # continuous, never wrapped to ~0..


def test_altitude_passes_through_without_geoid():
    assert project_wgs84(55.8, 37.4, 172.25, ProjectionConfig()).z_m == 172.25


@pytest.mark.parametrize(
    "lat, lon", [(math.nan, 37.0), (55.0, math.inf), (95.0, 37.0), (55.0, 200.0)]
)
def test_projection_rejects_bad_input(lat, lon):
    with pytest.raises(ValueError):
        project_wgs84(lat, lon, 0.0, ProjectionConfig())


def test_central_meridian_easting():
    p = project_wgs84(55.0, 39.0, 0.0, ProjectionConfig())  # zone 37 central meridian
    assert p.x_m == pytest.approx(500000.0 - 300000.0, abs=1e-6)


# --- pathgraph on synthetic routes ----------------------------------------------------------


def test_pose_at_interpolates_and_checks_bounds():
    g = PathGraph({"r": straight_route(0.0, dz=0.02)})
    pose = g.pose_at("r", 10.5)
    assert pose.x_m == pytest.approx(10.5 / math.sqrt(1 + 0.02**2), rel=1e-3)
    assert pose.grade == pytest.approx(0.02, abs=1e-6)
    assert pose.yaw_rad == pytest.approx(0.0)
    with pytest.raises(OutOfGraphError):
        g.pose_at("r", -0.1)
    with pytest.raises(OutOfGraphError):
        g.pose_at("r", g.length_m("r") + 0.1)
    with pytest.raises(KeyError):
        g.pose_at("nope", 1.0)


def test_project_pose_at_roundtrip_error_below_5cm():
    g = two_track_graph()
    for s in (0.0, 0.3, 17.25, 50.0, 99.9, 100.0):
        pose = g.pose_at("fwd", s)
        m = g.project(pose.x_m, pose.y_m, pose.yaw_rad)
        assert m.status == STATUS_MATCHED
        assert m.route_id == "fwd"
        assert abs(m.s_m - s) < 0.05


def test_neighbouring_tracks_separated_by_heading():
    g = two_track_graph()
    # mid-way between tracks, closer to the forward one, heading forward
    m = g.project(50.0, 1.7, yaw_rad=0.0)
    assert (m.route_id, m.status) == ("fwd", STATUS_MATCHED)
    m = g.project(50.0, 1.7, yaw_rad=math.pi)
    assert m.route_id == "back"


def test_ambiguous_without_heading_between_tracks():
    g = two_track_graph()
    m = g.project(50.0, 1.85)  # equidistant, no heading
    assert m.status == STATUS_AMBIGUOUS
    assert g.project(50.0, 0.2).status == STATUS_MATCHED


def test_out_of_graph_is_not_clamped_to_route_end():
    g = two_track_graph()
    far = g.project(50.0, 50.0)
    assert far.status == STATUS_OUT_OF_GRAPH and far.s_m is None and far.route_id is None
    beyond_end = g.project(140.0, 0.0, yaw_rad=0.0)  # 40 m past the end of "fwd"
    assert beyond_end.status == STATUS_OUT_OF_GRAPH and beyond_end.s_m is None
    assert g.project(-1.0, 0.0, yaw_rad=0.0, route_id="fwd").status == STATUS_MATCHED


def test_cross_track_sign_left_positive():
    g = PathGraph({"r": straight_route(0.0)})
    assert g.project(20.0, 1.0).cross_track_m == pytest.approx(1.0)
    assert g.project(20.0, -1.0).cross_track_m == pytest.approx(-1.0)


def test_route_id_filter_and_unknown_route():
    g = two_track_graph()
    assert g.project(50.0, 3.7, route_id="fwd").status == STATUS_MATCHED
    with pytest.raises(KeyError):
        g.project(50.0, 0.0, route_id="nope")


def test_body_pose_yaw_from_bogie_chord_and_start_extrapolation():
    g = PathGraph({"r": straight_route(math.pi / 4)})
    body = g.body_pose_at("r", 20.0)
    assert body.yaw_rad == pytest.approx(math.pi / 4, abs=1e-9)
    at_start = g.body_pose_at("r", 0.0)  # rear bogie is before the first point
    assert at_start.yaw_rad == pytest.approx(math.pi / 4, abs=1e-9)
    with pytest.raises(OutOfGraphError):
        g.body_pose_at("r", g.length_m("r") + 1.0)


def test_invalid_pathgraph_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        PathGraph({})
    with pytest.raises(ValueError):
        PathGraph({"r": [{"x": 0, "y": 0, "z": 0, "tang": 0, "curv": 0}]})
    with pytest.raises(ValueError):
        PathGraph({"r": [{"x": 0, "y": 0}, {"x": 1, "y": 0}]})
    with pytest.raises(ValueError):
        PathGraph({"r": straight_route()[:1] + straight_route()[:1]})  # duplicate point
    (tmp_path / "bad.json").write_text("{not json", encoding="utf-8")
    with pytest.raises(ValueError):
        PathGraph.from_directory(tmp_path)
    with pytest.raises(ValueError):
        PathGraph.from_directory(tmp_path / "missing")


# --- antennas -------------------------------------------------------------------------------


def antenna_points(base_x, base_y, base_z, yaw, geometry=TramGeometry()):
    def at(antenna):
        c, s = math.cos(yaw), math.sin(yaw)
        return MapPoint(
            base_x + c * antenna[0] - s * antenna[1],
            base_y + s * antenna[0] + c * antenna[1],
            base_z + antenna[2],
        )

    return at(geometry.antenna_1_xyz_m), at(geometry.antenna_2_xyz_m)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi, -2.0, 3.0])
def test_dual_antenna_recovers_base_link(yaw):
    geometry = TramGeometry()
    p1, p2 = antenna_points(1000.0, 2000.0, 170.0, yaw)
    pose = base_link_from_dual_antenna(p1, p2, geometry)
    assert (pose.x_m, pose.y_m, pose.z_m) == pytest.approx((1000.0, 2000.0, 170.0), abs=1e-9)
    assert math.cos(pose.yaw_rad - yaw) == pytest.approx(1.0, abs=1e-12)
    assert pose.baseline_error_m == pytest.approx(0.0, abs=1e-9)
    assert math.hypot(p2.x_m - p1.x_m, p2.y_m - p1.y_m) == pytest.approx(12.436, abs=1e-9)


def test_dual_antenna_reports_baseline_error_and_rejects_coincident():
    geometry = TramGeometry()
    p1, p2 = antenna_points(0.0, 0.0, 0.0, 0.3)
    stretched = MapPoint(p2.x_m + 0.5 * math.cos(0.3), p2.y_m + 0.5 * math.sin(0.3), p2.z_m)
    assert base_link_from_dual_antenna(p1, stretched, geometry).baseline_error_m == pytest.approx(0.5)
    with pytest.raises(ValueError):
        base_link_from_dual_antenna(p1, p1, geometry)
    with pytest.raises(ValueError):
        base_link_from_dual_antenna(MapPoint(math.nan, 0, 0), p2, geometry)


@pytest.mark.parametrize("yaw", [0.0, math.pi / 2, math.pi])
def test_single_antenna_uses_known_yaw(yaw):
    geometry = TramGeometry()
    _, p2 = antenna_points(10.0, -5.0, 3.0, yaw)
    base = base_link_from_single_antenna(p2, yaw, geometry.antenna_2_xyz_m)
    assert (base.x_m, base.y_m, base.z_m) == pytest.approx((10.0, -5.0, 3.0), abs=1e-9)


# --- real Pathgraph ------------------------------------------------------------------------


@pytest.fixture(scope="module")
def real_graph():
    if not PATHGRAPH.exists():
        pytest.skip("dataset/Pathgraph not available")
    return PathGraph.from_directory(PATHGRAPH)


def test_real_pathgraph_routes_and_ascii_ids(real_graph):
    assert real_graph.route_ids == ("shchukinskaya-tallinskaya", "tallinskaya-shchukinskaya")
    for rid in real_graph.route_ids:
        assert rid.isascii()
        assert 4600 < real_graph.length_m(rid) < 4800


def test_real_pathgraph_roundtrip_and_direction(real_graph):
    for rid in real_graph.route_ids:
        for s in (0.0, 1234.5, 3000.0, real_graph.length_m(rid) - 1.0):
            pose = real_graph.pose_at(rid, s)
            m = real_graph.project(pose.x_m, pose.y_m, pose.yaw_rad)
            assert m.status == STATUS_MATCHED
            assert m.route_id == rid
            assert abs(m.s_m - s) < 0.05
        assert abs(real_graph.pose_at(rid, 2000.0).grade) < 0.06  # slopes stay under ~4%
