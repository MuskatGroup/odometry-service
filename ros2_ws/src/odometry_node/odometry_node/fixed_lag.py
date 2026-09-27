"""Fixed-lag event-time commit with a disposable current-time prediction."""

from __future__ import annotations

import copy

TRANSPORT_REASONS = {
    "DUPLICATE",
    "GAP_RESET",
    "INVALID_INPUT",
    "OUT_OF_ORDER",
    "QUEUE_OVERFLOW",
}


def advance_fixed_lag(estimator, now_ns: int, reorder_window_ns: int):
    """Commit only the safe history and return a prediction stamped at ``now_ns``.

    The committed estimator remains behind wall/ROS time, so events delayed by transport can
    still be inserted in timestamp order. A deep-copied preview consumes the currently known
    queue and predicts to now; publishing it does not close the committed history.
    """
    if estimator.t is None:
        raise ValueError("Estimator must be initialized before fixed-lag advance")
    if not isinstance(now_ns, int) or not isinstance(reorder_window_ns, int):
        raise TypeError("Fixed-lag timestamps must be integer nanoseconds")
    if now_ns < estimator.t or reorder_window_ns < 0:
        raise ValueError("Invalid current time or reorder window")

    commit_ns = max(estimator.t, now_ns - reorder_window_ns)
    committed = estimator.advance_to(commit_ns)
    # ``seen`` can contain thousands of deduplication keys. Prediction never enqueues new
    # events, so sharing this read-only cache avoids an O(history) copy on every 50 Hz tick.
    # ``grade_provider`` is shared read-only too: besides being wasted work to copy every tick,
    # a caller-supplied bound method (unlike a lambda, which copy.deepcopy never recurses into)
    # would drag the method's whole owning object into the copy — e.g. a ROS node, whose
    # publishers/locks cannot be pickled at all, turning a routine tick into a crash.
    shared = {id(estimator.seen): estimator.seen, id(estimator.grade_provider): estimator.grade_provider}
    preview = copy.deepcopy(estimator, shared)
    result = preview.advance_to(now_ns)
    for reason in committed.reason_codes:
        if reason in TRANSPORT_REASONS and reason not in result.reason_codes:
            result.reason_codes.append(reason)
    result.reason_codes.sort()
    return result, commit_ns

