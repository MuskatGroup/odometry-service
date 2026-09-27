"""Directed track geometry from the organizers' Pathgraph JSON files.

Each file holds one directed route (``points`` with x, y, z, tang, curv, 1 m step). ``s`` is the
accumulated 3D length along the route. Projection never clamps a far-away point onto a route end:
outside the gate the result is ``OUT_OF_GRAPH``.
"""

from __future__ import annotations

import json
import math
from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path

STATUS_MATCHED = "MATCHED"
STATUS_AMBIGUOUS = "AMBIGUOUS"
STATUS_OUT_OF_GRAPH = "OUT_OF_GRAPH"

_CYRILLIC = dict(
    zip(
        "абвгдеёжзийклмнопрстуфхцчшщъыьэюя",
        [
            "a", "b", "v", "g", "d", "e", "e", "zh", "z", "i", "i", "k", "l", "m", "n", "o", "p", "r",
            "s", "t", "u", "f", "kh", "ts", "ch", "sh", "shch", "", "y", "", "e", "yu", "ya",
        ],
    )
)

# Stable ASCII route ids, curated per the organizers' two known filenames (see docs
# `19-pathgraph-audit-and-integration.md`, section 5, requirement 9). Deliberately NOT derived
# from the filename text at load time: requirement 9 demands ids that do not depend on the
# filename, so a rename, re-casing, or whitespace change in the organizers' files must never
# change the route id. Keyed by a normalized (casefolded, whitespace-collapsed) file stem so
# trivial spelling variants of the same two files still resolve, while anything genuinely new
# is refused rather than silently given an invented id (see `route_id_for_file`).
_ROUTE_ID_BY_STEM = {
    "таллинская - щукинская": "tallinskaya_to_shchukinskaya",
    "щукинская - таллинская": "shchukinskaya_to_tallinskaya",
}


def _normalize_stem(stem: str) -> str:
    return " ".join(stem.casefold().split())


class OutOfGraphError(ValueError):
    """``s`` outside the route."""


@dataclass(frozen=True)
class TrackPose:
    route_id: str
    s_m: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    curvature_inv_m: float
    grade: float


@dataclass(frozen=True)
class TrackMatch:
    route_id: str | None
    s_m: float | None
    cross_track_m: float | None  # signed, positive to the left of the travel direction
    heading_error_rad: float | None
    status: str


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def route_id_from_name(name: str) -> str:
    """Mechanical, filename-derived transliteration, e.g. 'таллинская - щукинская' becomes
    'tallinskaya-shchukinskaya'.

    This is NOT a stable route id (requirement 9 explicitly forbids one that depends on the
    filename) and ``PathGraph.from_directory`` never uses it to name a route on its own. It
    exists only so an error about an unmapped file can suggest a candidate id to add to
    ``_ROUTE_ID_BY_STEM``.
    """
    out = []
    for ch in name.lower():
        if ch in _CYRILLIC:
            out.append(_CYRILLIC[ch])
        elif ch.isascii() and ch.isalnum():
            out.append(ch)
        else:
            out.append("-")
    text = "-".join(part for part in "".join(out).split("-") if part)
    if not text:
        raise ValueError(f"cannot build route id from {name!r}")
    return text


def route_id_for_file(file: Path) -> str:
    """Stable ASCII route id for a Pathgraph file, independent of the exact filename text.

    Looks the file's (normalized) stem up in the curated ``_ROUTE_ID_BY_STEM`` table so the id
    never silently changes if the organizers rename, re-case, or re-space a file. A stem that
    is not in the table is refused rather than given an invented, filename-derived id, since
    that would just recreate the instability requirement 9 forbids one level up: extend the
    table instead.
    """
    stem = _normalize_stem(file.stem)
    try:
        return _ROUTE_ID_BY_STEM[stem]
    except KeyError:
        suggestion = route_id_from_name(file.stem)
        raise ValueError(
            f"{file.name}: file stem {stem!r} is not in the curated route id table "
            f"(_ROUTE_ID_BY_STEM); add it there with a stable id (e.g. {suggestion!r} is the "
            "mechanical transliteration, but pick the organizers' proposed id if this is one of "
            "the two known routes) rather than deriving one from the filename"
        ) from None


def _ordered_points_from_document(document: object, file_name: str) -> list[dict]:
    """Validate top-level ``points``/``paths`` and return points in ``point_indices`` order.

    Per the spec (section 5, requirements 1-3) the route's point order is defined by
    concatenating ``point_indices`` across every entry of ``paths``, not by assuming
    ``points`` is already in route order.
    """
    if not isinstance(document, dict):
        raise ValueError(f"{file_name}: top-level document must be an object")
    points = document.get("points")
    paths = document.get("paths")
    if not isinstance(points, list):
        raise ValueError(f"{file_name}: 'points' must be a list")
    if not isinstance(paths, list):
        raise ValueError(f"{file_name}: 'paths' must be a list")
    if not paths:
        raise ValueError(f"{file_name}: 'paths' must contain at least one entry")
    ordered_indices: list[int] = []
    for path_i, entry in enumerate(paths):
        if not isinstance(entry, dict):
            raise ValueError(f"{file_name}: paths[{path_i}] must be an object")
        point_indices = entry.get("point_indices")
        if not isinstance(point_indices, list):
            raise ValueError(f"{file_name}: paths[{path_i}].point_indices must be a list")
        for index in point_indices:
            if isinstance(index, bool) or not isinstance(index, int):
                raise ValueError(
                    f"{file_name}: paths[{path_i}].point_indices contains a non-integer index "
                    f"({index!r})"
                )
            if index < 0 or index >= len(points):
                raise ValueError(
                    f"{file_name}: paths[{path_i}].point_indices index {index} out of bounds "
                    f"for {len(points)} points"
                )
        ordered_indices.extend(point_indices)
    if not ordered_indices:
        raise ValueError(f"{file_name}: 'paths' produced an empty point sequence")
    return [points[i] for i in ordered_indices]


class _Route:
    def __init__(self, route_id: str, points: list[dict]) -> None:
        if len(points) < 2:
            raise ValueError(f"route {route_id}: need at least 2 points")
        self.route_id = route_id
        try:
            self.x = [float(p["x"]) for p in points]
            self.y = [float(p["y"]) for p in points]
            self.z = [float(p["z"]) for p in points]
            self.yaw = [_wrap(float(p["tang"])) for p in points]
            self.curv = [float(p["curv"]) for p in points]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"route {route_id}: bad point ({exc!r})") from exc
        for series in (self.x, self.y, self.z, self.yaw, self.curv):
            if not all(math.isfinite(v) for v in series):
                raise ValueError(f"route {route_id}: non-finite value")
        self.s = [0.0]
        self.seg_len = []  # horizontal length of each segment
        for i in range(len(self.x) - 1):
            dx, dy, dz = self.x[i + 1] - self.x[i], self.y[i + 1] - self.y[i], self.z[i + 1] - self.z[i]
            length = math.sqrt(dx * dx + dy * dy + dz * dz)
            if length <= 0.0:
                raise ValueError(f"route {route_id}: duplicate point at index {i}")
            self.s.append(self.s[-1] + length)
            self.seg_len.append(math.hypot(dx, dy))
        self.grade = self._grades()

    def _grades(self, half_window_m: float = 5.0) -> list[float]:
        """dz / horizontal distance over a +-5 m window (raw z is noisy point to point)."""
        n = len(self.x)
        cum_h = [0.0]
        for length in self.seg_len:
            cum_h.append(cum_h[-1] + length)
        grades = []
        lo = hi = 0
        for i in range(n):
            while cum_h[i] - cum_h[lo] > half_window_m:
                lo += 1
            while hi < n - 1 and cum_h[hi] - cum_h[i] < half_window_m:
                hi += 1
            dh = cum_h[hi] - cum_h[lo]
            grades.append((self.z[hi] - self.z[lo]) / dh if dh > 0.0 else 0.0)
        return grades

    @property
    def length(self) -> float:
        return self.s[-1]

    def pose_at(self, s_m: float) -> TrackPose:
        if not math.isfinite(s_m) or s_m < 0.0 or s_m > self.length:
            raise OutOfGraphError(f"s={s_m} outside route {self.route_id} [0, {self.length:.3f}]")
        i = min(bisect_right(self.s, s_m) - 1, len(self.s) - 2)
        f = (s_m - self.s[i]) / (self.s[i + 1] - self.s[i])

        def lerp(values: list[float]) -> float:
            return values[i] + f * (values[i + 1] - values[i])

        yaw = self.yaw[i] + f * _wrap(self.yaw[i + 1] - self.yaw[i])
        return TrackPose(
            route_id=self.route_id,
            s_m=s_m,
            x_m=lerp(self.x),
            y_m=lerp(self.y),
            z_m=lerp(self.z),
            yaw_rad=_wrap(yaw),
            curvature_inv_m=lerp(self.curv),
            grade=lerp(self.grade),
        )


class PathGraph:
    def __init__(
        self,
        routes: dict[str, list[dict]],
        max_cross_track_m: float = 6.0,
        ambiguity_margin_m: float = 1.0,
        max_heading_error_rad: float = math.pi / 2,
        end_overshoot_m: float = 2.0,
        cell_size_m: float = 25.0,
    ) -> None:
        if not routes:
            raise ValueError("no routes")
        if max_cross_track_m <= 0.0 or cell_size_m <= 0.0:
            raise ValueError("gate and cell size must be positive")
        self._routes = {rid: _Route(rid, pts) for rid, pts in sorted(routes.items())}
        self.max_cross_track_m = max_cross_track_m
        self.ambiguity_margin_m = ambiguity_margin_m
        self.max_heading_error_rad = max_heading_error_rad
        self.end_overshoot_m = end_overshoot_m
        self._cell = cell_size_m
        self._index: dict[tuple[int, int], list[tuple[str, int]]] = {}
        for rid, route in self._routes.items():
            for i in range(len(route.x) - 1):
                self._insert(rid, i, route)

    @classmethod
    def from_directory(cls, path: str | Path, **kwargs) -> PathGraph:
        directory = Path(path)
        files = sorted(directory.glob("*.json"))
        if not files:
            raise ValueError(f"no *.json files in {directory}")
        routes: dict[str, list[dict]] = {}
        for file in files:
            try:
                document = json.loads(file.read_text(encoding="utf-8"))
            except (OSError, ValueError) as exc:
                raise ValueError(f"{file.name}: invalid Pathgraph file ({exc})") from exc
            ordered_points = _ordered_points_from_document(document, file.name)
            rid = route_id_for_file(file)
            if rid in routes:
                raise ValueError(f"duplicate route id {rid!r}")
            routes[rid] = ordered_points
        return cls(routes, **kwargs)

    def _insert(self, rid: str, i: int, route: _Route) -> None:
        g = self.max_cross_track_m
        x0, x1 = sorted((route.x[i], route.x[i + 1]))
        y0, y1 = sorted((route.y[i], route.y[i + 1]))
        cell = self._cell
        for ix in range(math.floor((x0 - g) / cell), math.floor((x1 + g) / cell) + 1):
            for iy in range(math.floor((y0 - g) / cell), math.floor((y1 + g) / cell) + 1):
                self._index.setdefault((ix, iy), []).append((rid, i))

    @property
    def route_ids(self) -> tuple[str, ...]:
        return tuple(self._routes)

    def length_m(self, route_id: str) -> float:
        return self._routes[route_id].length

    def pose_at(self, route_id: str, s_m: float) -> TrackPose:
        return self._routes[route_id].pose_at(s_m)

    def body_pose_at(self, route_id: str, front_s_m: float, bogie_base_m: float = 7.55) -> TrackPose:
        """Pose of ``base_link`` (front bogie) with yaw from the bogie chord. If the rear bogie is
        still before the route start it is extrapolated backwards along the first tangent."""
        route = self._routes[route_id]
        front = route.pose_at(front_s_m)
        rear_s = front_s_m - bogie_base_m
        if rear_s >= 0.0:
            rear = route.pose_at(rear_s)
            rx, ry = rear.x_m, rear.y_m
        else:
            first = route.pose_at(0.0)
            rx = first.x_m + rear_s * math.cos(first.yaw_rad)
            ry = first.y_m + rear_s * math.sin(first.yaw_rad)
        dx, dy = front.x_m - rx, front.y_m - ry
        yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else front.yaw_rad
        return TrackPose(
            route_id=route_id,
            s_m=front_s_m,
            x_m=front.x_m,
            y_m=front.y_m,
            z_m=front.z_m,
            yaw_rad=yaw,
            curvature_inv_m=front.curvature_inv_m,
            grade=front.grade,
        )

    def project(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float | None = None,
        route_id: str | None = None,
    ) -> TrackMatch:
        if not (math.isfinite(x_m) and math.isfinite(y_m)):
            raise ValueError("x and y must be finite")
        if yaw_rad is not None and not math.isfinite(yaw_rad):
            raise ValueError("yaw must be finite")
        if route_id is not None and route_id not in self._routes:
            raise KeyError(route_id)
        cell = (math.floor(x_m / self._cell), math.floor(y_m / self._cell))
        best_per_route: dict[str, tuple[float, float, float, float | None]] = {}
        nearest = math.inf
        for rid, i in self._index.get(cell, ()):
            if route_id is not None and rid != route_id:
                continue
            candidate = self._project_segment(self._routes[rid], i, x_m, y_m)
            if candidate is None:
                continue
            distance, s_m, signed, seg_yaw = candidate
            nearest = min(nearest, distance)
            if distance > self.max_cross_track_m:
                continue
            herr = None if yaw_rad is None else _wrap(yaw_rad - seg_yaw)
            if herr is not None and abs(herr) > self.max_heading_error_rad:
                continue  # track of the opposite direction
            current = best_per_route.get(rid)
            if current is None or distance < current[0]:
                best_per_route[rid] = (distance, s_m, signed, herr)
        if not best_per_route:
            return TrackMatch(
                None, None, nearest if math.isfinite(nearest) else None, None, STATUS_OUT_OF_GRAPH
            )
        ranked = sorted(best_per_route.items(), key=lambda item: item[1][0])
        rid, (distance, s_m, signed, herr) = ranked[0]
        ambiguous = len(ranked) > 1 and ranked[1][1][0] - distance < self.ambiguity_margin_m
        return TrackMatch(rid, s_m, signed, herr, STATUS_AMBIGUOUS if ambiguous else STATUS_MATCHED)

    def _project_segment(
        self, route: _Route, i: int, x_m: float, y_m: float
    ) -> tuple[float, float, float, float] | None:
        ax, ay, bx, by = route.x[i], route.y[i], route.x[i + 1], route.y[i + 1]
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 0.0:
            return None
        seg_h = math.sqrt(length_sq)
        t = ((x_m - ax) * dx + (y_m - ay) * dy) / length_sq
        if (i == 0 and t < 0.0 and -t * seg_h > self.end_overshoot_m) or (
            i == len(route.x) - 2 and t > 1.0 and (t - 1.0) * seg_h > self.end_overshoot_m
        ):
            return None
        tc = min(1.0, max(0.0, t))
        px, py = ax + tc * dx, ay + tc * dy
        distance = math.hypot(x_m - px, y_m - py)
        s_m = route.s[i] + tc * (route.s[i + 1] - route.s[i])
        side = dx * (y_m - ay) - dy * (x_m - ax)  # cross product: positive = left of travel
        return distance, s_m, math.copysign(distance, side), math.atan2(dy, dx)
