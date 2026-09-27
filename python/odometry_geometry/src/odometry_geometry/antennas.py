"""GNSS antenna -> ``base_link`` transforms.

``base_link`` is the centre of the front bogie; x points forward, z up (REP-103). Antenna offsets are
given in that frame. Yaw is measured counter-clockwise from map x (east); pitch/roll are ignored.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .projection import MapPoint


@dataclass(frozen=True)
class TramGeometry:
    bogie_base_m: float = 7.55
    antenna_1_xyz_m: tuple[float, float, float] = (-9.873, 0.0, 3.0)
    antenna_2_xyz_m: tuple[float, float, float] = (2.563, 0.0, 3.0)


@dataclass(frozen=True)
class BaseLinkPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    baseline_error_m: float | None


def _rotate(dx: float, dy: float, yaw_rad: float) -> tuple[float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return c * dx - s * dy, s * dx + c * dy


def _check_finite(*values: float) -> None:
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError("values must be finite numbers")


def base_link_from_dual_antenna(
    point_1: MapPoint, point_2: MapPoint, geometry: TramGeometry
) -> BaseLinkPose:
    """Yaw from the antenna baseline (antenna 1 -> antenna 2 must match the mounting), position
    averaged over both antennas. ``baseline_error_m`` = measured - expected horizontal baseline."""
    _check_finite(point_1.x_m, point_1.y_m, point_1.z_m, point_2.x_m, point_2.y_m, point_2.z_m)
    a1, a2 = geometry.antenna_1_xyz_m, geometry.antenna_2_xyz_m
    expected_dx, expected_dy = a2[0] - a1[0], a2[1] - a1[1]
    expected = math.hypot(expected_dx, expected_dy)
    if expected <= 0.0:
        raise ValueError("antennas must not coincide")
    dx, dy = point_2.x_m - point_1.x_m, point_2.y_m - point_1.y_m
    measured = math.hypot(dx, dy)
    if measured < 1e-6:
        raise ValueError("measured antenna positions coincide: yaw is undefined")
    yaw = math.atan2(dy, dx) - math.atan2(expected_dy, expected_dx)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    estimates = []
    for point, antenna in ((point_1, a1), (point_2, a2)):
        ox, oy = _rotate(antenna[0], antenna[1], yaw)
        estimates.append((point.x_m - ox, point.y_m - oy, point.z_m - antenna[2]))
    return BaseLinkPose(
        x_m=(estimates[0][0] + estimates[1][0]) / 2.0,
        y_m=(estimates[0][1] + estimates[1][1]) / 2.0,
        z_m=(estimates[0][2] + estimates[1][2]) / 2.0,
        yaw_rad=yaw,
        baseline_error_m=measured - expected,
    )


def base_link_from_single_antenna(
    point: MapPoint, yaw_rad: float, antenna_xyz_m: tuple[float, float, float]
) -> MapPoint:
    """``base_link`` position from one antenna when the body yaw is known from elsewhere."""
    _check_finite(point.x_m, point.y_m, point.z_m, yaw_rad, *antenna_xyz_m)
    ox, oy = _rotate(antenna_xyz_m[0], antenna_xyz_m[1], yaw_rad)
    return MapPoint(point.x_m - ox, point.y_m - oy, point.z_m - antenna_xyz_m[2])
