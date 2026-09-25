"""Reusable dataframe helpers for organizer dataset notebooks.

GNSS data exposed here is ground truth for offline analysis only. It must not
be used as an estimator input during evaluation.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml
from pyproj import CRS, Transformer
from read_bag import DATA_ROOT, create_typestore, iter_records, resolve_bag_path
from rosbags.highlevel import AnyReader
from rosbags.interfaces.typing import Nodetype

TOPIC_COLUMNS = {
    "/vehicle/driver_position_cmd": "control_count",
    "/vehicle/front_bogie_velocity": "front_wheel_count",
    "/vehicle/rear_bogie_velocity": "rear_wheel_count",
    "/sensing/gnss/master/fix": "master_fix_count",
    "/sensing/gnss/master/vel": "master_velocity_count",
    "/sensing/gnss/rover/fix": "rover_fix_count",
    "/sensing/gnss/rover/vel": "rover_velocity_count",
}

RECORD_COLUMNS = [
    "topic",
    "recorded_ns",
    "header_ns",
    "recording_delay_ns",
    "frame_id",
    "kind",
    "position",
    "wheel_id",
    "velocity_mps",
    "velocity_finite",
    "receiver",
    "status",
    "service",
    "has_fix",
    "coordinates_finite",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "position_covariance",
    "position_covariance_type",
    "velocity_components_finite",
    "velocity_x_mps",
    "velocity_y_mps",
    "velocity_z_mps",
    "angular_components_finite",
    "angular_x_radps",
    "angular_y_radps",
    "angular_z_radps",
    "horizontal_speed_mps",
    "speed_3d_mps",
]


@dataclass(frozen=True)
class BagFrames:
    """Normalized tables belonging to one bag."""

    bag_id: str
    bag_path: Path
    origin_ns: int
    records: pd.DataFrame
    control: pd.DataFrame
    wheels: pd.DataFrame
    gnss_fix: pd.DataFrame
    gnss_velocity: pd.DataFrame


def _topic_counts(info: dict[str, Any]) -> dict[str, int]:
    counts = {column: 0 for column in TOPIC_COLUMNS.values()}
    for entry in info.get("topics_with_message_count", []):
        topic = entry.get("topic_metadata", {}).get("name")
        column = TOPIC_COLUMNS.get(topic)
        if column:
            counts[column] = int(entry.get("message_count", 0))
    return counts


def read_bag_metadata(bag_path: Path) -> dict[str, Any]:
    """Read and flatten one rosbag2 metadata file."""
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found: {metadata_path}")

    payload = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
    info = payload["rosbag2_bagfile_information"]
    counts = _topic_counts(info)
    relative_files = info.get("relative_file_paths", [])
    size_bytes = sum(
        (bag_path / filename).stat().st_size
        for filename in relative_files
        if (bag_path / filename).is_file()
    )

    duration_ns = int(info["duration"]["nanoseconds"])
    start_ns = int(info["starting_time"]["nanoseconds_since_epoch"])
    row: dict[str, Any] = {
        "bag_id": bag_path.name,
        "vehicle_id": bag_path.name.split("_", maxsplit=1)[0],
        "duration_s": duration_ns / 1e9,
        "start_ns": start_ns,
        "message_count": int(info.get("message_count", 0)),
        "size_mib": size_bytes / (1024**2),
        **counts,
    }
    row["has_master_gnss"] = (
        row["master_fix_count"] > 0 and row["master_velocity_count"] > 0
    )
    row["has_rover_gnss"] = (
        row["rover_fix_count"] > 0 and row["rover_velocity_count"] > 0
    )
    row["has_any_gnss"] = row["has_master_gnss"] or row["has_rover_gnss"]
    row["has_both_gnss"] = row["has_master_gnss"] and row["has_rover_gnss"]
    return row


def scan_dataset(data_root: Path = DATA_ROOT) -> pd.DataFrame:
    """Build a metadata-only inventory without decoding db3 messages."""
    rows = [
        read_bag_metadata(path)
        for path in sorted(data_root.iterdir())
        if path.is_dir() and (path / "metadata.yaml").is_file()
    ]
    inventory = pd.DataFrame(rows)
    if inventory.empty:
        raise ValueError(f"No bag directories found under {data_root}")
    return inventory.sort_values(["vehicle_id", "start_ns", "bag_id"]).reset_index(
        drop=True
    )


def _field_type_name(descriptor: tuple[Any, Any]) -> str:
    node, value = descriptor
    if node == Nodetype.BASE:
        return str(value[0])
    if node == Nodetype.NAME:
        return str(value)
    if node == Nodetype.ARRAY:
        child, length = value
        return f"{_field_type_name(child)}[{length}]"
    if node == Nodetype.SEQUENCE:
        child, length = value
        suffix = "" if length == 0 else f"<={length}"
        return f"sequence<{_field_type_name(child)}>{suffix}"
    return str(value)


def _flatten_type_fields(
    typestore: Any,
    message_type: str,
    prefix: str = "",
) -> list[tuple[str, str]]:
    fields: list[tuple[str, str]] = []
    for field_name, descriptor in typestore.fielddefs[message_type][1]:
        path = f"{prefix}.{field_name}" if prefix else field_name
        node, nested_type = descriptor
        if node == Nodetype.NAME and nested_type in typestore.fielddefs:
            fields.extend(_flatten_type_fields(typestore, nested_type, path))
        else:
            fields.append((path, _field_type_name(descriptor)))
    return fields


def bag_topic_schema(bag: str | Path) -> pd.DataFrame:
    """List every topic and recursively expanded ROS message field."""
    bag_path = resolve_bag_path(str(bag))
    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")

    typestore = create_typestore()
    rows: list[dict[str, Any]] = []
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        for connection in sorted(reader.connections, key=lambda item: item.topic):
            for field, field_type in _flatten_type_fields(
                typestore, connection.msgtype
            ):
                rows.append(
                    {
                        "topic": connection.topic,
                        "message_type": connection.msgtype,
                        "message_count": connection.msgcount,
                        "field": field,
                        "field_type": field_type,
                    }
                )
    return pd.DataFrame(rows)


def _add_time_columns(records: pd.DataFrame, origin_ns: int) -> pd.DataFrame:
    result = records.copy()
    result["time_s"] = (result["header_ns"] - origin_ns) / 1e9
    result["recorded_time_s"] = (result["recorded_ns"] - origin_ns) / 1e9
    result["delay_ms"] = result["recording_delay_ns"] / 1e6
    return result


def add_local_gnss_coordinates(
    fixes: pd.DataFrame,
    preferred_receiver: str = "master",
) -> pd.DataFrame:
    """Add local metric coordinates using an azimuthal-equidistant projection."""
    result = fixes.copy()
    result[["local_x_m", "local_y_m", "local_z_m"]] = np.nan
    valid = (
        result["has_fix"].eq(True)
        & result["coordinates_finite"].eq(True)
        & result["latitude_deg"].notna()
        & result["longitude_deg"].notna()
    )
    if not valid.any():
        return result

    preferred = result[valid & (result["receiver"] == preferred_receiver)]
    reference = preferred.iloc[0] if not preferred.empty else result[valid].iloc[0]
    latitude_0 = float(reference["latitude_deg"])
    longitude_0 = float(reference["longitude_deg"])
    altitude_0 = float(reference["altitude_m"] or 0.0)
    local_crs = CRS.from_proj4(
        f"+proj=aeqd +lat_0={latitude_0} +lon_0={longitude_0} "
        "+datum=WGS84 +units=m +no_defs"
    )
    transformer = Transformer.from_crs("EPSG:4326", local_crs, always_xy=True)
    x_values, y_values = transformer.transform(
        result.loc[valid, "longitude_deg"].to_numpy(),
        result.loc[valid, "latitude_deg"].to_numpy(),
    )
    result.loc[valid, "local_x_m"] = x_values
    result.loc[valid, "local_y_m"] = y_values
    result.loc[valid, "local_z_m"] = (
        result.loc[valid, "altitude_m"].astype(float) - altitude_0
    )
    result.attrs["reference"] = {
        "receiver": reference["receiver"],
        "latitude_deg": latitude_0,
        "longitude_deg": longitude_0,
        "altitude_m": altitude_0,
    }
    return result


def load_bag_frames(bag: str | Path) -> BagFrames:
    """Decode one bag and split it into analysis-ready dataframes."""
    bag_path = resolve_bag_path(str(bag))
    records = pd.DataFrame(iter_records(bag_path)).reindex(columns=RECORD_COLUMNS)
    if records.empty:
        raise ValueError(f"No supported messages found in {bag_path}")

    origin_ns = int(records["header_ns"].min())
    records = _add_time_columns(records, origin_ns)
    control = records[records["kind"] == "control"].copy()
    control["normalized_position"] = control["position"] / 15.0
    wheels = records[records["kind"] == "wheel_velocity"].copy()
    gnss_fix = add_local_gnss_coordinates(
        records[records["kind"] == "gnss_fix"].copy()
    )
    gnss_velocity = records[records["kind"] == "gnss_velocity"].copy()

    return BagFrames(
        bag_id=bag_path.name,
        bag_path=bag_path,
        origin_ns=origin_ns,
        records=records,
        control=control,
        wheels=wheels,
        gnss_fix=gnss_fix,
        gnss_velocity=gnss_velocity,
    )


def stream_timing_summary(frames: BagFrames) -> pd.DataFrame:
    """Summarize rates, time reversals, duplicates, and record/header offsets."""
    records = frames.records.copy()
    records["stream"] = records["kind"]
    records.loc[records["kind"] == "wheel_velocity", "stream"] += (
        ":" + records.loc[records["kind"] == "wheel_velocity", "wheel_id"]
    )
    gnss_mask = records["kind"].str.startswith("gnss")
    if gnss_mask.any():
        records.loc[gnss_mask, "stream"] += (
            ":" + records.loc[gnss_mask, "receiver"]
        )

    rows: list[dict[str, Any]] = []
    for stream, group in records.groupby("stream", sort=True):
        header_diffs = group["header_ns"].diff()
        positive_diffs = header_diffs[header_diffs > 0]
        median_period_s = (
            float(positive_diffs.median() / 1e9) if not positive_diffs.empty else np.nan
        )
        rows.append(
            {
                "stream": stream,
                "count": len(group),
                "header_span_s": (group["header_ns"].max() - group["header_ns"].min())
                / 1e9,
                "median_rate_hz": (
                    1.0 / median_period_s if median_period_s > 0 else np.nan
                ),
                "median_delay_ms": group["delay_ms"].median(),
                "p95_delay_ms": group["delay_ms"].quantile(0.95),
                "negative_delay_count": int((group["delay_ms"] < 0).sum()),
                "header_reversal_count": int((header_diffs < 0).sum()),
                "duplicate_header_count": int(group["header_ns"].duplicated().sum()),
            }
        )
    return pd.DataFrame(rows).set_index("stream")


def build_speed_comparison(
    frames: BagFrames,
    tolerance_ms: float = 150.0,
) -> pd.DataFrame:
    """Synchronize wheel speed magnitude with GNSS horizontal speed for analysis."""
    if frames.wheels.empty or frames.gnss_velocity.empty:
        return pd.DataFrame()

    wheel_wide = frames.wheels.pivot_table(
        index="header_ns",
        columns="wheel_id",
        values="velocity_mps",
        aggfunc="last",
    ).sort_index()
    wheel_wide["wheel_median_mps"] = wheel_wide.median(axis=1, skipna=True)
    wheel_wide["wheel_speed_magnitude_mps"] = wheel_wide["wheel_median_mps"].abs()
    if {"front_bogie", "rear_bogie"}.issubset(wheel_wide.columns):
        wheel_wide["wheel_disagreement_mps"] = (
            wheel_wide["front_bogie"] - wheel_wide["rear_bogie"]
        ).abs()

    truth_wide = frames.gnss_velocity.pivot_table(
        index="header_ns",
        columns="receiver",
        values="horizontal_speed_mps",
        aggfunc="last",
    ).sort_index()
    truth_wide["gnss_speed_mps"] = truth_wide.median(axis=1, skipna=True)

    comparison = pd.merge_asof(
        wheel_wide.reset_index().sort_values("header_ns"),
        truth_wide.reset_index().sort_values("header_ns"),
        on="header_ns",
        direction="nearest",
        tolerance=int(tolerance_ms * 1e6),
    )
    comparison["time_s"] = (comparison["header_ns"] - frames.origin_ns) / 1e9
    comparison["speed_error_mps"] = (
        comparison["wheel_speed_magnitude_mps"] - comparison["gnss_speed_mps"]
    )
    return comparison


def speed_error_metrics(comparison: pd.DataFrame) -> pd.Series:
    """Calculate simple descriptive wheel-vs-GNSS metrics."""
    if comparison.empty:
        return pd.Series(dtype=float)
    paired = comparison.dropna(
        subset=["wheel_speed_magnitude_mps", "gnss_speed_mps"]
    )
    if paired.empty:
        return pd.Series(dtype=float)

    error = paired["speed_error_mps"]
    return pd.Series(
        {
            "paired_samples": len(paired),
            "coverage": len(paired) / len(comparison),
            "mae_mps": error.abs().mean(),
            "rmse_mps": np.sqrt(np.mean(np.square(error))),
            "bias_mps": error.mean(),
            "p95_abs_error_mps": error.abs().quantile(0.95),
            "correlation": paired[
                ["wheel_speed_magnitude_mps", "gnss_speed_mps"]
            ].corr().iloc[0, 1],
        }
    )


def candidate_duplicate_groups(inventory: pd.DataFrame) -> pd.DataFrame:
    """Find metadata-identical runs that require content-level deduplication."""
    signature = [
        "vehicle_id",
        "duration_s",
        "message_count",
        *TOPIC_COLUMNS.values(),
    ]
    candidates = inventory[inventory.duplicated(signature, keep=False)].copy()
    if candidates.empty:
        return candidates
    candidates["candidate_group"] = candidates.groupby(signature, dropna=False).ngroup()
    return candidates.sort_values(["candidate_group", "bag_id"])
