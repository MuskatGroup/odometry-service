"""GNSS-derived reference speed / along-track position for offline evaluation and identification.

GNSS is *truth for offline work only*: nothing here may feed the runtime estimator, and wheel speed
is never used as a target. Standard library + ``odometry_geometry`` only, so it is unit-testable.

Pipeline: two GNSS velocity streams -> consensus speed on a common grid; two fix streams ->
``base_link`` (dual antenna) -> Pathgraph projection -> ``s_ref``; then a check of the consensus
speed against the derivative of the smoothed position.
"""

from __future__ import annotations

import math
import sys
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python/odometry_geometry/src"))

from odometry_geometry import (  # noqa: E402
    STATUS_MATCHED,
    MapPoint,
    PathGraph,
    ProjectionConfig,
    TramGeometry,
    base_link_from_dual_antenna,
    project_wgs84,
)

# reason codes
OK = "OK"
SINGLE_SOURCE = "SINGLE_SOURCE"
SOURCES_DISAGREE = "SOURCES_DISAGREE"
POSITION_MISMATCH = "POSITION_MISMATCH"
NO_VELOCITY = "NO_VELOCITY"


@dataclass(frozen=True)
class GnssFix:
    stamp_ns: int
    receiver: str  # "master" (antenna 1) or "rover" (antenna 2)
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    status: int  # NavSatStatus: -1 no fix, >=0 fix


@dataclass(frozen=True)
class GnssVelocity:
    stamp_ns: int
    receiver: str
    vx_mps: float
    vy_mps: float
    vz_mps: float


@dataclass(frozen=True)
class ReferenceConfig:
    max_velocity_gap_s: float = 0.25  # do not interpolate a receiver across a longer gap
    max_source_diff_mps: float = 0.3  # master vs rover speed tolerance
    speed_floor_mps: float = 0.03
    single_source_sigma_mps: float = 0.15
    max_pair_dt_s: float = 0.06  # master/rover fixes closer than this are one epoch
    max_fix_gap_s: float = 0.5  # do not interpolate s across a longer gap
    max_baseline_error_m: float = 0.5
    position_window_s: float = 1.0
    max_position_mismatch_mps: float = 1.0
    moving_threshold_mps: float = 0.5  # below this the sign of ds/dt is not trusted


@dataclass(frozen=True)
class ReferenceSample:
    stamp_ns: int
    v_ref_mps: float | None
    s_ref_m: float | None
    route_id: str | None
    uncertainty_mps: float | None
    reason: str


def _interp(times: list[float], values: list[float], t: float, max_gap_s: float) -> float | None:
    i = bisect_left(times, t)
    if i < len(times) and times[i] == t:
        return values[i]
    if i == 0 or i == len(times):
        return None
    t0, t1 = times[i - 1], times[i]
    if t1 - t0 > max_gap_s:
        return None
    return values[i - 1] + (values[i] - values[i - 1]) * (t - t0) / (t1 - t0)


class SpeedReferenceBuilder:
    def __init__(
        self,
        graph: PathGraph,
        config: ReferenceConfig | None = None,
        projection: ProjectionConfig | None = None,
        geometry: TramGeometry | None = None,
    ) -> None:
        self.graph = graph
        self.config = config or ReferenceConfig()
        self.projection = projection or ProjectionConfig()
        self.geometry = geometry or TramGeometry()

    # -- position ----------------------------------------------------------------------------

    def positions(self, fixes: list[GnssFix]) -> tuple[list[float], list[float], list[str]]:
        """Matched along-track positions as (times_s, s_m, route_id) of ``base_link``."""
        cfg = self.config
        valid = [f for f in fixes if f.status >= 0 and self._finite(f)]
        masters = sorted((f for f in valid if f.receiver == "master"), key=lambda f: f.stamp_ns)
        rovers = sorted((f for f in valid if f.receiver == "rover"), key=lambda f: f.stamp_ns)
        rover_times = [f.stamp_ns / 1e9 for f in rovers]
        times: list[float] = []
        along: list[float] = []
        routes: list[str] = []
        for master in masters:
            t = master.stamp_ns / 1e9
            j = bisect_left(rover_times, t)
            candidates = [k for k in (j - 1, j) if 0 <= k < len(rovers)]
            if not candidates:
                continue
            k = min(candidates, key=lambda idx: abs(rover_times[idx] - t))
            if abs(rover_times[k] - t) > cfg.max_pair_dt_s:
                continue
            p1 = self._map_point(master)
            p2 = self._map_point(rovers[k])
            try:
                pose = base_link_from_dual_antenna(p1, p2, self.geometry)
            except ValueError:
                continue
            if pose.baseline_error_m is None or abs(pose.baseline_error_m) > cfg.max_baseline_error_m:
                continue
            match = self.graph.project(pose.x_m, pose.y_m, pose.yaw_rad)
            if match.status != STATUS_MATCHED or match.s_m is None or match.route_id is None:
                continue
            times.append((t + rover_times[k]) / 2.0)
            along.append(match.s_m)
            routes.append(match.route_id)
        return times, along, routes

    def _map_point(self, fix: GnssFix) -> MapPoint:
        return project_wgs84(fix.latitude_deg, fix.longitude_deg, fix.altitude_m, self.projection)

    @staticmethod
    def _finite(fix: GnssFix) -> bool:
        return all(math.isfinite(v) for v in (fix.latitude_deg, fix.longitude_deg, fix.altitude_m))

    # -- reference ---------------------------------------------------------------------------

    def build(self, fixes: list[GnssFix], velocities: list[GnssVelocity]) -> list[ReferenceSample]:
        cfg = self.config
        streams: dict[str, tuple[list[float], list[float]]] = {}
        for receiver in ("master", "rover"):
            rows = sorted(
                (v for v in velocities if v.receiver == receiver and math.isfinite(v.vx_mps + v.vy_mps)),
                key=lambda v: v.stamp_ns,
            )
            streams[receiver] = (
                [v.stamp_ns / 1e9 for v in rows],
                [math.hypot(v.vx_mps, v.vy_mps) for v in rows],
            )
        stamps_ns = sorted({v.stamp_ns for v in velocities if v.receiver in streams})
        if not stamps_ns:
            return []

        pos_t, pos_s, pos_route = self.positions(fixes)
        pos_speed = self._position_speed(pos_t, pos_s)
        speed_t = [x for x, _ in pos_speed]
        speed_v = [y for _, y in pos_speed]

        samples = []
        for stamp_ns in stamps_ns:
            t = stamp_ns / 1e9
            values = [
                v
                for v in (
                    _interp(times, speeds, t, cfg.max_velocity_gap_s) for times, speeds in streams.values()
                )
                if v is not None
            ]
            s_ref = _interp(pos_t, pos_s, t, cfg.max_fix_gap_s)
            route = pos_route[min(bisect_left(pos_t, t), len(pos_route) - 1)] if pos_route else None
            if s_ref is None:
                route = None
            speed, sigma, reason = self._consensus(values)
            derivative = _interp(speed_t, speed_v, t, cfg.max_fix_gap_s)
            if speed is not None and derivative is not None:
                if abs(derivative) > cfg.moving_threshold_mps and derivative < 0.0:
                    speed = -speed  # travelling backwards along the directed route
                if abs(speed - derivative) > cfg.max_position_mismatch_mps:
                    reason = POSITION_MISMATCH
                    sigma = max(sigma or 0.0, abs(speed - derivative) / 2.0)
            samples.append(ReferenceSample(stamp_ns, speed, s_ref, route, sigma, reason))
        return samples

    def _consensus(self, values: list[float]) -> tuple[float | None, float | None, str]:
        cfg = self.config
        if not values:
            return None, None, NO_VELOCITY
        if len(values) == 1:
            return values[0], cfg.single_source_sigma_mps, SINGLE_SOURCE
        a, b = values
        if abs(a - b) > cfg.max_source_diff_mps:
            return None, abs(a - b) / 2.0, SOURCES_DISAGREE
        return (a + b) / 2.0, max(abs(a - b) / 2.0, cfg.speed_floor_mps), OK

    def _position_speed(self, times: list[float], along: list[float]) -> list[tuple[float, float]]:
        """ds/dt from the positions over +-position_window_s (a slope, so noise averages out)."""
        window = self.config.position_window_s
        out = []
        lo = hi = 0
        for i, t in enumerate(times):
            while times[lo] < t - window:
                lo += 1
            while hi < len(times) - 1 and times[hi + 1] <= t + window:
                hi += 1
            if hi > lo and times[hi] - times[lo] >= window:
                out.append((t, (along[hi] - along[lo]) / (times[hi] - times[lo])))
        return out


def acceleration(samples: list[ReferenceSample], half_window_s: float = 0.5) -> list[float | None]:
    """a_ref from centred differences of v_ref over +-half_window_s; None where v_ref is missing."""
    times = [s.stamp_ns / 1e9 for s in samples]
    speeds = [s.v_ref_mps for s in samples]
    known_times = [t for t, v in zip(times, speeds, strict=True) if v is not None]
    known_speeds = [v for v in speeds if v is not None]
    out: list[float | None] = []
    for t in times:
        before = _interp(known_times, known_speeds, t - half_window_s, 0.3)
        after = _interp(known_times, known_speeds, t + half_window_s, 0.3)
        out.append(None if before is None or after is None else (after - before) / (2 * half_window_s))
    return out


def coverage(samples: list[ReferenceSample]) -> float:
    return sum(s.v_ref_mps is not None for s in samples) / len(samples) if samples else 0.0
