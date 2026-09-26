"""Loading and validating the organizer-provided Pathgraph JSON files."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

DATASET_ROOT = Path(__file__).resolve().parents[1]
PATHGRAPH_ROOT = DATASET_ROOT / "Pathgraph"

ROUTE_FILE_NAMES = {
    "tallinskaya_to_shchukinskaya": "таллинская - щукинская.json",
    "shchukinskaya_to_tallinskaya": "щукинская - таллинская.json",
}

REQUIRED_POINT_FIELDS = ("x", "y", "z", "tang", "curv")


@dataclass(frozen=True)
class PathGraphRoute:
    """One ordered route with geometry derived from the source JSON."""

    route_id: str
    source_path: Path
    points: pd.DataFrame

    @property
    def length_m(self) -> float:
        return float(self.points["s_m"].iloc[-1])

    @property
    def length_xy_m(self) -> float:
        return float(self.points["s_xy_m"].iloc[-1])


def normalize_angle(angle_rad: np.ndarray) -> np.ndarray:
    """Normalize angles to [-pi, pi]."""

    return np.arctan2(np.sin(angle_rad), np.cos(angle_rad))


def _resolve_route_path(route: str | Path, pathgraph_root: Path) -> tuple[str, Path]:
    if isinstance(route, Path) or str(route).lower().endswith(".json"):
        source_path = Path(route)
        if not source_path.is_absolute():
            source_path = pathgraph_root / source_path
        route_id = next(
            (candidate for candidate, name in ROUTE_FILE_NAMES.items() if name == source_path.name),
            source_path.stem,
        )
        return route_id, source_path

    route_id = str(route)
    try:
        source_path = pathgraph_root / ROUTE_FILE_NAMES[route_id]
    except KeyError as exc:
        known = ", ".join(sorted(ROUTE_FILE_NAMES))
        raise ValueError(f"Unknown route_id {route_id!r}. Known route IDs: {known}") from exc
    return route_id, source_path


def load_pathgraph(
    route: str | Path,
    *,
    pathgraph_root: Path = PATHGRAPH_ROOT,
    grade_window_points: int = 31,
) -> PathGraphRoute:
    """Load one Pathgraph file into an ordered, validated DataFrame.

    ``route`` may be a stable route ID or a JSON path. ``s_m`` is the cumulative
    three-dimensional arc length and ``s_xy_m`` is the planar arc length.
    """

    route_id, source_path = _resolve_route_path(route, pathgraph_root)
    if not source_path.is_file():
        raise FileNotFoundError(f"Pathgraph file not found: {source_path}")

    with source_path.open("r", encoding="utf-8") as source:
        payload = json.load(source)

    points = payload.get("points")
    paths = payload.get("paths")
    if not isinstance(points, list) or not points:
        raise ValueError(f"{source_path}: 'points' must be a non-empty list")
    if not isinstance(paths, list) or len(paths) != 1:
        raise ValueError(f"{source_path}: exactly one item in 'paths' is expected")

    point_indices = paths[0].get("point_indices")
    if not isinstance(point_indices, list) or len(point_indices) < 2:
        raise ValueError(f"{source_path}: 'point_indices' must contain at least two indices")
    if any(not isinstance(index, int) or index < 0 or index >= len(points) for index in point_indices):
        raise ValueError(f"{source_path}: 'point_indices' contains an invalid index")

    ordered_points: list[dict[str, float | int]] = []
    for order, point_index in enumerate(point_indices):
        point = points[point_index]
        if not isinstance(point, dict):
            raise ValueError(f"{source_path}: point {point_index} must be an object")
        missing = [field for field in REQUIRED_POINT_FIELDS if field not in point]
        if missing:
            raise ValueError(f"{source_path}: point {point_index} misses fields: {missing}")
        try:
            values = {field: float(point[field]) for field in REQUIRED_POINT_FIELDS}
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{source_path}: point {point_index} contains a non-numeric value") from exc
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError(f"{source_path}: point {point_index} contains a non-finite value")
        ordered_points.append({"order": order, "point_index": point_index, **values})

    frame = pd.DataFrame.from_records(ordered_points)
    xyz = frame[["x", "y", "z"]].to_numpy(dtype=float)
    xy = xyz[:, :2]
    segment_xyz = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    segment_xy = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    if np.any(segment_xyz <= 0.0):
        bad_segment = int(np.flatnonzero(segment_xyz <= 0.0)[0])
        raise ValueError(f"{source_path}: zero-length segment at ordered index {bad_segment}")

    frame["s_m"] = np.concatenate(([0.0], np.cumsum(segment_xyz)))
    frame["s_xy_m"] = np.concatenate(([0.0], np.cumsum(segment_xy)))
    frame["yaw_rad"] = normalize_angle(frame["tang"].to_numpy(dtype=float))
    frame["curvature_inv_m"] = frame["curv"].to_numpy(dtype=float)

    raw_grade = np.gradient(frame["z"].to_numpy(dtype=float), frame["s_m"].to_numpy(dtype=float))
    window = max(1, int(grade_window_points))
    if window % 2 == 0:
        window += 1
    frame["grade_raw"] = raw_grade
    frame["grade"] = (
        pd.Series(raw_grade).rolling(window=window, center=True, min_periods=1).median().to_numpy()
    )
    frame["route_id"] = route_id

    return PathGraphRoute(route_id=route_id, source_path=source_path, points=frame)


def load_all_pathgraphs(*, pathgraph_root: Path = PATHGRAPH_ROOT) -> dict[str, PathGraphRoute]:
    """Load both organizer-provided directed routes."""

    return {
        route_id: load_pathgraph(route_id, pathgraph_root=pathgraph_root)
        for route_id in ROUTE_FILE_NAMES
    }


def pathgraph_summary(routes: dict[str, PathGraphRoute] | None = None) -> pd.DataFrame:
    """Return a compact table useful in notebooks and validation reports."""

    selected = routes or load_all_pathgraphs()
    records = []
    for route in selected.values():
        frame = route.points
        records.append(
            {
                "route_id": route.route_id,
                "point_count": len(frame),
                "length_m": route.length_m,
                "length_xy_m": route.length_xy_m,
                "x_min": frame["x"].min(),
                "x_max": frame["x"].max(),
                "y_min": frame["y"].min(),
                "y_max": frame["y"].max(),
                "z_min": frame["z"].min(),
                "z_max": frame["z"].max(),
            }
        )
    return pd.DataFrame.from_records(records).set_index("route_id")
