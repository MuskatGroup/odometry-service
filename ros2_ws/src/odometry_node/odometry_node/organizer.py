"""Pure conversions for the organizer's ROS wire contract."""

import math


def controller_position_to_u(position):
    value = int(position)
    if not -15 <= value <= 15:
        raise ValueError("Controller position must be in [-15, 15]")
    return value / 15.0


def wheel_kmh_to_mps(velocity):
    value = float(velocity)
    if not math.isfinite(value) or value < 0:
        raise ValueError("Wheel velocity must be finite and non-negative")
    return value / 3.6
