"""Read organizer ROS 2 bags (sqlite3 + CDR) without ROS or extra packages.

Only the message layouts used by the dataset are decoded:

* ``tram_vehicle_msgs/msg/VelocitySensor``           Header + float64 velocity
* ``tram_vehicle_msgs/msg/DriverControllerCommand``  Header + int8 position
* ``geometry_msgs/msg/TwistStamped``                 Header + Twist (6 x float64)
* ``sensor_msgs/msg/NavSatFix``                      Header + NavSatStatus + lat, lon, alt

Values are returned exactly as recorded (no unit conversion, no clamping): what the numbers mean is
decided by the caller. Times are integer nanoseconds; ``header`` is the measurement time written by
the sensor, ``record`` is when rosbag2 stored the message.
"""

from __future__ import annotations

import sqlite3
import struct
from dataclasses import dataclass, field
from pathlib import Path

WHEEL_TOPICS = {
    "/vehicle/front_bogie_velocity": "front",
    "/vehicle/rear_bogie_velocity": "rear",
}
CONTROL_TOPIC = "/vehicle/driver_position_cmd"
GNSS_FIX_TOPICS = {
    "/sensing/gnss/master/fix": "master",
    "/sensing/gnss/rover/fix": "rover",
}
GNSS_VELOCITY_TOPICS = {
    "/sensing/gnss/master/vel": "master",
    "/sensing/gnss/rover/vel": "rover",
}


class BagError(ValueError):
    pass


@dataclass
class BagData:
    bag_id: str
    vehicle_id: str
    # (header_ns, record_ns, wheel_id, raw value as recorded)
    wheels: list[tuple[int, int, str, float]] = field(default_factory=list)
    # (header_ns, record_ns, position as recorded, -15..15)
    controls: list[tuple[int, int, int]] = field(default_factory=list)
    # (header_ns, receiver, vx, vy, vz)
    gnss_velocity: list[tuple[int, str, float, float, float]] = field(default_factory=list)
    # (header_ns, receiver, latitude_deg, longitude_deg, altitude_m, status)
    gnss_fix: list[tuple[int, str, float, float, float, int]] = field(default_factory=list)
    bad_messages: int = 0
    topic_counts: dict[str, int] = field(default_factory=dict)


def _header(data: bytes) -> tuple[int, int]:
    """Return (stamp_ns, offset of the first byte after Header) for a CDR message."""
    if len(data) < 16:
        raise BagError("message too short")
    sec, nanosec = struct.unpack_from("<iI", data, 4)
    (length,) = struct.unpack_from("<I", data, 12)
    end = 16 + length
    if length > 4096 or end > len(data):
        raise BagError("invalid frame_id length")
    return sec * 1_000_000_000 + nanosec, end


def _aligned(offset: int, size: int) -> int:
    """CDR aligns primitives relative to the start of the payload after the 4-byte encapsulation."""
    return (offset - 4 + size - 1) // size * size + 4


def parse_velocity(data: bytes) -> tuple[int, float]:
    stamp, end = _header(data)
    (value,) = struct.unpack_from("<d", data, _aligned(end, 8))
    return stamp, value


def parse_control(data: bytes) -> tuple[int, int]:
    stamp, end = _header(data)
    (position,) = struct.unpack_from("<b", data, end)
    return stamp, position


def parse_twist(data: bytes) -> tuple[int, tuple[float, ...]]:
    stamp, end = _header(data)
    return stamp, struct.unpack_from("<6d", data, _aligned(end, 8))


def parse_fix(data: bytes) -> tuple[int, int, float, float, float]:
    """NavSatFix: Header, NavSatStatus(int8 status, uint16 service), lat, lon, alt (float64)."""
    stamp, end = _header(data)
    (status,) = struct.unpack_from("<b", data, end)
    lat, lon, alt = struct.unpack_from("<3d", data, _aligned(_aligned(end + 1, 2) + 2, 8))
    return stamp, status, lat, lon, alt


def find_database(bag: Path) -> Path:
    bag = Path(bag)
    if bag.is_file() and bag.suffix == ".db3":
        return bag
    databases = sorted(bag.glob("*.db3"))
    if not databases:
        raise BagError(f"No .db3 file in {bag}")
    return databases[0]


def read_bag(bag: str | Path) -> BagData:
    bag = Path(bag)
    database = find_database(bag)
    bag_id = database.stem.removesuffix("_0")
    data = BagData(bag_id=bag_id, vehicle_id=bag_id.split("_", maxsplit=1)[0])
    connection = sqlite3.connect(f"file:{database.as_posix()}?mode=ro", uri=True)
    try:
        topics = {row[0]: row[1] for row in connection.execute("SELECT id, name FROM topics")}
        for topic_id, record_ns, blob in connection.execute(
            "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
        ):
            name = topics.get(topic_id)
            if name is None:
                continue
            data.topic_counts[name] = data.topic_counts.get(name, 0) + 1
            payload = bytes(blob)
            try:
                if name in WHEEL_TOPICS:
                    stamp, value = parse_velocity(payload)
                    data.wheels.append((stamp, record_ns, WHEEL_TOPICS[name], value))
                elif name == CONTROL_TOPIC:
                    stamp, position = parse_control(payload)
                    data.controls.append((stamp, record_ns, position))
                elif name in GNSS_FIX_TOPICS:
                    stamp, status, lat, lon, alt = parse_fix(payload)
                    data.gnss_fix.append((stamp, GNSS_FIX_TOPICS[name], lat, lon, alt, status))
                elif name in GNSS_VELOCITY_TOPICS:
                    stamp, twist = parse_twist(payload)
                    data.gnss_velocity.append((stamp, GNSS_VELOCITY_TOPICS[name], *twist[:3]))
            except (BagError, struct.error):
                data.bad_messages += 1
    finally:
        connection.close()
    return data
