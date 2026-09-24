"""Replaceable fusion policies; a baseline is not a slip detector."""

from dataclasses import dataclass
import statistics
from .core import InputError, number


@dataclass(frozen=True)
class FusionResult:
    speed_mps: float
    variance: float
    used_channels: tuple[str, ...]


def median_strategy(events, settings):
    values = [event.value for event in events]
    center = statistics.median(values)
    mad = statistics.median(abs(value - center) for value in values)
    variance = max(number(settings.get("variance_floor", 0.25), "variance_floor"), (1.4826 * mad) ** 2)
    return FusionResult(center, variance, tuple(event.channel_id for event in events))


def selected_channel_strategy(events, settings):
    channel = settings.get("channel")
    match = next((event for event in events if event.channel_id == channel), None)
    if match is None:
        return None
    return FusionResult(match.value, number(settings.get("variance_floor", 0.25), "variance_floor"), (channel,))


STRATEGIES = {"median": median_strategy, "selected_channel": selected_channel_strategy}


def register_strategy(name, strategy):
    if name in STRATEGIES or not callable(strategy):
        raise InputError(f"Cannot register strategy: {name}")
    STRATEGIES[name] = strategy
