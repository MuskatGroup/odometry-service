# Contracts 0.2

The runtime schema is defined by Python dataclasses in odometry_core.types and ROS messages
in odometry_msgs. Canonical JSON events have kind=control|wheel and decimal-string stamp_ns.
Samples and source profiles live here. Profiles are safe YAML, not executable expressions.

Version 0.2 adds uncertainty_available and has_estimate. Missing numerical uncertainty is
null in JSON, or flagged unavailable in ROS. The wheel-hold baseline NEVER claims calibrated
uncertainty; it publishes the extended estimate but not nav_msgs/Odometry.

seq is monotonic within a control stream or individual wheel stream in one run.
Reconnections/restarts with reset sequence numbers require a new run and estimator reset.
time.clock names the shared clock domain. offset_ns is an explicit fixed alignment;
there is no inferred clock synchronization. Live profiles need timestamps on the ROS clock.
