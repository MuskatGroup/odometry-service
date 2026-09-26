"""Narrow adapter to the independently developed odometry_geometry package."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class GnssMapObservation:
    route_id: str
    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    cross_track_m: float
    baseline_error_m: float | None


class GeometryAdapter:
    def __init__(self, directory):
        try:
            import odometry_geometry as geometry
        except ImportError as exc:
            raise RuntimeError(
                "odometry_geometry is required by the production runtime; merge the geometry package"
            ) from exc
        self.geometry = geometry
        self.graph = geometry.PathGraph.from_directory(directory)
        self.config = geometry.ProjectionConfig()
        self.tram = geometry.TramGeometry()

    def pose_at(self, route_id, s_m):
        return self.graph.pose_at(route_id, s_m)

    def body_pose_at(self, route_id, s_m):
        return self.graph.body_pose_at(route_id, s_m, self.tram.bogie_base_m)

    def grade_at(self, route_id, s_m):
        return self.graph.pose_at(route_id, s_m).grade

    def observe_dual(self, master_fix, rover_fix, route_id=None):
        first = self.geometry.project_wgs84(*master_fix, self.config)
        second = self.geometry.project_wgs84(*rover_fix, self.config)
        base = self.geometry.base_link_from_dual_antenna(first, second, self.tram)
        match = self.graph.project(base.x_m, base.y_m, base.yaw_rad, route_id)
        if match.status != "MATCHED" or match.route_id is None or match.s_m is None:
            return None
        return GnssMapObservation(
            match.route_id,
            match.s_m,
            base.x_m,
            base.y_m,
            base.z_m,
            base.yaw_rad,
            float(match.cross_track_m or 0.0),
            base.baseline_error_m,
        )

    def observe_single(self, fix, antenna_xyz_m, route_id=None):
        antenna = self.geometry.project_wgs84(*fix, self.config)
        preliminary = self.graph.project(antenna.x_m, antenna.y_m, route_id=route_id)
        if preliminary.status != "MATCHED" or preliminary.route_id is None or preliminary.s_m is None:
            return None
        track = self.graph.pose_at(preliminary.route_id, preliminary.s_m)
        base = self.geometry.base_link_from_single_antenna(antenna, track.yaw_rad, antenna_xyz_m)
        match = self.graph.project(base.x_m, base.y_m, track.yaw_rad, preliminary.route_id)
        if match.status != "MATCHED" or match.s_m is None:
            return None
        return GnssMapObservation(
            preliminary.route_id,
            match.s_m,
            base.x_m,
            base.y_m,
            base.z_m,
            track.yaw_rad,
            float(match.cross_track_m or 0.0),
            None,
        )
