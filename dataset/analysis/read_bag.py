"""Inspect and normalize the odometry dataset ROS 2 bags.

The vehicle topics are candidate inputs for the estimator. GNSS records are
exported as reference data only and must not be fed to the reserve odometry
algorithm during evaluation.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from collections.abc import Iterator
from pathlib import Path
from typing import Any

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

DATASET_ROOT = Path(__file__).resolve().parents[1]
DATA_ROOT = DATASET_ROOT / "data"
MSG_DIR = DATASET_ROOT / "tram_vehicle_msgs" / "msg"
DEFAULT_BAG_ID = "30618_0d865417"

CONTROL_TOPIC = "/vehicle/driver_position_cmd"
WHEEL_TOPICS = {
    "/vehicle/front_bogie_velocity": "front_bogie",
    "/vehicle/rear_bogie_velocity": "rear_bogie",
}
GNSS_FIX_TOPICS = {
    "/sensing/gnss/master/fix": "master",
    "/sensing/gnss/rover/fix": "rover",
}
GNSS_VELOCITY_TOPICS = {
    "/sensing/gnss/master/vel": "master",
    "/sensing/gnss/rover/vel": "rover",
}
ALLOWED_TOPICS = {
    CONTROL_TOPIC,
    *WHEEL_TOPICS,
    *GNSS_FIX_TOPICS,
    *GNSS_VELOCITY_TOPICS,
}


def create_typestore():
    """Create a ROS 2 Humble typestore with the dataset's custom messages."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    custom_types: dict[str, Any] = {}

    for message_name in ("VelocitySensor", "DriverControllerCommand"):
        definition_path = MSG_DIR / f"{message_name}.msg"
        if not definition_path.is_file():
            raise FileNotFoundError(
                f"Custom message definition not found: {definition_path}"
            )

        custom_types.update(
            get_types_from_msg(
                definition_path.read_text(encoding="utf-8"),
                f"tram_vehicle_msgs/msg/{message_name}",
            )
        )

    typestore.register(custom_types)
    return typestore


def timestamp_ns(header: Any) -> int:
    """Convert a ROS Header timestamp to integer nanoseconds."""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def finite_number(value: Any) -> float | None:
    """Return a JSON-safe float, mapping ROS NaN/Inf markers to null."""
    number = float(value)
    return number if math.isfinite(number) else None


def normalize_message(
    topic: str,
    recorded_ns: int,
    message: Any,
) -> dict[str, Any]:
    """Convert one supported ROS message to a JSON-serializable record."""
    header_ns = timestamp_ns(message.header)
    common: dict[str, Any] = {
        "topic": topic,
        "recorded_ns": int(recorded_ns),
        "header_ns": header_ns,
        "recording_delay_ns": int(recorded_ns) - header_ns,
        "frame_id": message.header.frame_id,
    }

    if topic == CONTROL_TOPIC:
        return {
            **common,
            "kind": "control",
            "position": int(message.position),
        }

    if topic in WHEEL_TOPICS:
        velocity = finite_number(message.velocity)
        return {
            **common,
            "kind": "wheel_velocity",
            "wheel_id": WHEEL_TOPICS[topic],
            "velocity_mps": velocity,
            "velocity_finite": velocity is not None,
        }

    if topic in GNSS_FIX_TOPICS:
        latitude = float(message.latitude)
        longitude = float(message.longitude)
        altitude = float(message.altitude)
        status = int(message.status.status)

        return {
            **common,
            "kind": "gnss_fix",
            "receiver": GNSS_FIX_TOPICS[topic],
            "status": status,
            "service": int(message.status.service),
            "has_fix": status >= 0,
            "coordinates_finite": all(
                math.isfinite(value) for value in (latitude, longitude, altitude)
            ),
            "latitude_deg": finite_number(latitude),
            "longitude_deg": finite_number(longitude),
            "altitude_m": finite_number(altitude),
            "position_covariance": [
                finite_number(value) for value in message.position_covariance
            ],
            "position_covariance_type": int(message.position_covariance_type),
        }

    if topic in GNSS_VELOCITY_TOPICS:
        linear = message.twist.linear
        angular = message.twist.angular
        velocity_x = float(linear.x)
        velocity_y = float(linear.y)
        velocity_z = float(linear.z)
        angular_x = float(angular.x)
        angular_y = float(angular.y)
        angular_z = float(angular.z)

        components_finite = all(
            math.isfinite(value) for value in (velocity_x, velocity_y, velocity_z)
        )
        angular_components_finite = all(
            math.isfinite(value) for value in (angular_x, angular_y, angular_z)
        )
        return {
            **common,
            "kind": "gnss_velocity",
            "receiver": GNSS_VELOCITY_TOPICS[topic],
            "velocity_components_finite": components_finite,
            "velocity_x_mps": finite_number(velocity_x),
            "velocity_y_mps": finite_number(velocity_y),
            "velocity_z_mps": finite_number(velocity_z),
            "angular_components_finite": angular_components_finite,
            "angular_x_radps": finite_number(angular_x),
            "angular_y_radps": finite_number(angular_y),
            "angular_z_radps": finite_number(angular_z),
            "horizontal_speed_mps": (
                math.hypot(velocity_x, velocity_y) if components_finite else None
            ),
            "speed_3d_mps": (
                math.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)
                if components_finite
                else None
            ),
        }

    raise ValueError(f"Unsupported topic: {topic}")


def iter_records(bag_path: Path) -> Iterator[dict[str, Any]]:
    """Yield normalized records from a bag in recorded-message order."""
    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")

    typestore = create_typestore()
    with AnyReader([bag_path], default_typestore=typestore) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in ALLOWED_TOPICS
        ]

        for connection, recorded_ns, rawdata in reader.messages(
            connections=connections
        ):
            message = reader.deserialize(rawdata, connection.msgtype)
            yield normalize_message(connection.topic, recorded_ns, message)


def resolve_bag_path(value: str) -> Path:
    """Resolve either a dataset bag id or an explicit directory path."""
    candidate = Path(value)
    if candidate.is_dir():
        return candidate.resolve()
    return DATA_ROOT / value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read supported vehicle and GNSS topics from a ROS 2 bag."
    )
    parser.add_argument(
        "bag",
        nargs="?",
        default=DEFAULT_BAG_ID,
        help="Bag id under dataset/data or an explicit bag directory path.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Stop after this many normalized messages.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.limit is not None and args.limit < 0:
        raise ValueError("--limit must be non-negative")

    bag_path = resolve_bag_path(args.bag)
    print(f"Reading bag: {bag_path}", file=sys.stderr)

    count = 0
    for record in iter_records(bag_path):
        if args.limit is not None and count >= args.limit:
            break
        print(json.dumps(record, ensure_ascii=False, allow_nan=False))
        count += 1

    print(f"Normalized messages: {count}", file=sys.stderr)


if __name__ == "__main__":
    main()
