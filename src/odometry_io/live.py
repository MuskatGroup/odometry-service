"""Live transport boundary with caller-owned clock and replayable tick journal."""

from collections import OrderedDict
import heapq
from .core import Event, InputError, integer
from .runner import HoldEstimator


class LiveSession:
    """Call from one event loop; serialize transport callbacks with clock callbacks.

    Ingestion never advances time. Call advance_to on an independent timer even
    during transport silence. Clock translation is explicitly owned by the caller.
    Journal actions before consuming them for reproducible failures and timeouts.
    """

    def __init__(self, profile, estimator=None, speed_timeout_s=0.5,
                 max_pending=10000, dedup_capacity=100000, journal=None):
        if max_pending <= 0 or dedup_capacity <= 0:
            raise InputError("Live queue/cache capacities must be positive")
        self.estimator = estimator if estimator is not None else HoldEstimator(profile, speed_timeout_s)
        self.max_pending, self.dedup_capacity = max_pending, dedup_capacity
        self.journal = journal
        self.pending = []
        self.seen = OrderedDict()
        self.serial = 0
        self.closed = None
        self.stats = {"duplicates": 0, "late_events": 0}

    def ingest(self, events):
        events = list(events)
        if self.journal:
            self.journal({"op": "ingest", "events": [e.to_dict() for e in events]})
        for event in events:
            if event.key in self.seen:
                if self.seen[event.key] != event:
                    raise InputError(f"Conflicting duplicate: {event.key}")
                self.stats["duplicates"] += 1
                continue
            if self.closed is not None and event.stamp_ns <= self.closed:
                self.stats["late_events"] += 1
                continue
            if len(self.pending) >= self.max_pending:
                raise InputError("Live input queue is full")
            self.seen[event.key] = event
            if len(self.seen) > self.dedup_capacity:
                self.seen.popitem(last=False)
            heapq.heappush(self.pending, (event.stamp_ns, self.serial, event))
            self.serial += 1

    def advance_to(self, stamp_ns):
        stamp_ns = integer(stamp_ns, "advance timestamp")
        if self.closed is not None and stamp_ns <= self.closed:
            raise InputError("Live ticks must increase; create a new session for a new clock epoch")
        if self.journal:
            self.journal({"op": "advance", "stamp_ns": str(stamp_ns)})
        while self.pending and self.pending[0][0] <= stamp_ns:
            stamp = self.pending[0][0]
            group = []
            while self.pending and self.pending[0][0] == stamp:
                group.append(heapq.heappop(self.pending)[2])
            self.estimator.ingest(stamp, sorted(group, key=lambda e: (e.kind != "control", e.channel_id or "", e.seq)))
        frame = self.estimator.advance_to(stamp_ns)
        self.closed = stamp_ns
        return frame


def replay_actions(actions, session):
    """Replay exact delivery/timer order, not only measurement timestamps."""
    for action in actions:
        if action.get("op") == "ingest":
            session.ingest(Event.from_dict(row) for row in action["events"])
        elif action.get("op") == "advance":
            yield session.advance_to(action["stamp_ns"])
        else:
            raise InputError("Unknown live journal action")
