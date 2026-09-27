"""Production reserve odometry node for the organizer's ROS contracts."""

from __future__ import annotations

import math
import resource
import threading
import time
from collections import deque
from dataclasses import replace

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from odometry_core import (
    AdaptiveOdometryEstimator,
    AlongTrackPositionCorrection,
    ControlSample,
    InitialState,
    LongitudinalVelocityCorrection,
    ModelConfig,
    WheelSample,
)
from odometry_io import load_model_config
from odometry_msgs.msg import LongitudinalEstimate
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from .conversion import estimate_message, ns, set_stamp
from .fixed_lag import advance_fixed_lag
from .geometry_adapter import GeometryAdapter
from .organizer import controller_position_to_u, percentile, wheel_kmh_to_mps

GNSS_POLICIES = {"disabled", "initialization_only", "intermittent"}
ANTENNAS = {
    "master": (-9.873, 0.0, 3.0),
    "rover": (2.563, 0.0, 3.0),
}
# Organizer topics are published RELIABLE with a tiny history depth (as little as 1): a reader that
# asks for best-effort gets that writer's best-effort path and can silently miss samples whenever our
# own processing (the 50 Hz tick, in particular) is briefly busy. Matching RELIABLE with a generous
# depth lets DDS retry/queue instead of dropping.
INPUT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=200,
)


class ReserveOdometryNode(Node):
    def __init__(self, geometry=None, model_config=None, parameter_overrides=None):
        super().__init__("reserve_odometry", parameter_overrides=parameter_overrides)
        # A single-threaded executor made the 50 Hz tick (which snapshots the whole estimator to
        # predict/publish) and message reception take turns on one worker: while the tick was busy,
        # DDS could not be drained, and a best-effort-treated organizer publisher (tiny history
        # depth) silently lost wheel/control samples that arrived in that window. Subscriptions get
        # their own reentrant group so receiving is never stuck behind the tick; the tick keeps a
        # group of its own so overlapping ticks still cannot run concurrently. Both groups touch the
        # same estimator, so every access is serialized through one lock regardless of which thread
        # the executor happens to run it on.
        self._lock = threading.Lock()
        self._subscription_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()
        defaults = {
            "vehicle_id": "default",
            "route_id": "",
            "s0": 0.0,
            "initial_v_mps": -1.0,  # -1: initialize from the first usable wheel group
            "gnss_policy": "disabled",
            "gnss_initialization_window_s": 5.0,
            "pathgraph_directory": "",
            "model_config": "",
            "allow_unidentified_model": False,
            "publish_rate_hz": 50.0,
            "processing_delay_ms": 300.0,
            "max_model_only_s": 10.0,
            "gnss_sync_tolerance_ms": 200.0,
            "gnss_cross_track_gate_m": 8.0,
            "gnss_baseline_error_gate_m": 1.0,
            "gnss_position_variance_m2": 4.0,
            "gnss_velocity_variance_m2ps2": 0.25,
            "cross_track_variance_m2": 4.0,
            "yaw_variance_rad2": 0.04,
            "run_id": "reserve-odometry",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.vehicle_id = self.get_parameter("vehicle_id").value
        self.route_id = self.get_parameter("route_id").value or None
        self.s0 = float(self.get_parameter("s0").value)
        self.gnss_policy = self.get_parameter("gnss_policy").value
        if self.gnss_policy not in GNSS_POLICIES:
            raise ValueError(f"gnss_policy must be one of {sorted(GNSS_POLICIES)}")
        initial_v = float(self.get_parameter("initial_v_mps").value)
        if not math.isfinite(initial_v) or (initial_v < 0 and initial_v != -1.0):
            raise ValueError("initial_v_mps must be non-negative or -1 for wheel initialization")
        self.gnss_window_s = float(self.get_parameter("gnss_initialization_window_s").value)
        if not math.isfinite(self.gnss_window_s) or self.gnss_window_s <= 0:
            raise ValueError("gnss_initialization_window_s must be finite and positive")
        self.input_origin_stamp = None
        profile = self.get_parameter("model_config").value
        allow_unidentified = bool(self.get_parameter("allow_unidentified_model").value)
        if model_config is None:
            if profile:
                model_config, _, selected = load_model_config(profile, self.vehicle_id)
                self.get_logger().info(f"Using identified model profile: {selected}")
            elif allow_unidentified:
                model_config = ModelConfig(max_model_only_s=self.get_parameter("max_model_only_s").value)
                self.get_logger().warning("Using uncalibrated built-in model parameters")
            else:
                raise ValueError("model_config is required; set allow_unidentified_model only for tests")
        model_config = replace(
            model_config,
            max_model_only_s=float(self.get_parameter("max_model_only_s").value),
            bootstrap_wheel_speed=initial_v == -1.0,
        )
        self.geometry = geometry
        if self.geometry is None:
            pathgraph = self.get_parameter("pathgraph_directory").value
            if pathgraph:
                self.geometry = GeometryAdapter(pathgraph)
            elif self.route_id or self.gnss_policy != "disabled":
                raise ValueError("pathgraph_directory is required for map positioning or GNSS")
        self.estimator = AdaptiveOdometryEstimator(
            model_config,
            # Plain lambda, not a bound method: copy.deepcopy treats function objects as atomic and
            # never recurses into their closure, so this survives advance_fixed_lag's preview copy.
            # A bound method is deep-copied through __self__, dragging in the whole Node (locks and
            # all) and raising "cannot pickle '_thread.lock' object".
            route_grade_provider=lambda route_id, s: self._grade_at(route_id, s),
        )
        self.model_config = model_config
        self.sequence = 0
        self.last_publish_stamp = None
        self.last_map_pose = None
        self.last_gnss_stamp = None
        self.last_perf_wall = time.perf_counter()
        self.last_perf_cpu = time.process_time()
        self.cpu_cores = 0.0
        self.pending_fixes = {}
        self.gnss_subscriptions = []
        self.gnss_closed = False
        self.gnss_initialized = False
        self.prefilter_rejected_position = 0
        self.prefilter_rejected_velocity = 0
        self.gnss_reasons = set()
        self.latency_ms = deque(maxlen=4096)
        self.pending_input_timing = deque(maxlen=4096)
        self.callback_received_ns = None
        self.latest_input_to_publication_ms = None
        self.input_lateness_ms = deque(maxlen=4096)
        self.wheel_lateness_ms = deque(maxlen=4096)
        self.late_input_count = 0
        self.too_late_input_count = 0
        self.crash_count = 0
        reorder_ms = float(self.get_parameter("processing_delay_ms").value)
        if not math.isfinite(reorder_ms) or reorder_ms < 0:
            raise ValueError("processing_delay_ms must be finite and non-negative")
        self.reorder_window_ns = round(reorder_ms * 1e6)
        self.filter_commit_stamp = None

        self.estimate_publisher = self.create_publisher(LongitudinalEstimate, "/odometry/estimate", 10)
        self.velocity_publisher = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self.position_publisher = self.create_publisher(Odometry, "/result/position", 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/odometry/diagnostics", 10
        )
        self.create_subscription(
            VelocitySensor,
            "/vehicle/front_bogie_velocity",
            self._guard("wheel front", lambda msg: self._wheel(msg, "front_bogie")),
            INPUT_QOS,
            callback_group=self._subscription_group,
        )
        self.create_subscription(
            VelocitySensor,
            "/vehicle/rear_bogie_velocity",
            self._guard("wheel rear", lambda msg: self._wheel(msg, "rear_bogie")),
            INPUT_QOS,
            callback_group=self._subscription_group,
        )
        self.create_subscription(
            DriverControllerCommand,
            "/vehicle/driver_position_cmd",
            self._guard("control", self._control),
            INPUT_QOS,
            callback_group=self._subscription_group,
        )
        if self.gnss_policy != "disabled":
            self._create_gnss_subscriptions()
        rate = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(rate) or rate < 10.0:
            raise ValueError("publish_rate_hz must be finite and >= 10")
        self.timer = self.create_timer(
            1.0 / rate, self._guard("tick", self._tick), callback_group=self._timer_group
        )

    def _guard(self, name, callback):
        """Wrap a callback so one bad message or a stray edge case cannot take the whole node down.

        The production run is judged over one long, uninterrupted bag; a crash forfeits everything
        after it, while a single skipped callback only costs one sample.
        """

        def _wrapped(*args, **kwargs):
            received_ns = time.perf_counter_ns()
            try:
                with self._lock:
                    self.callback_received_ns = received_ns
                    return callback(*args, **kwargs)
            except Exception:  # noqa: BLE001 - last-resort guard, logged in full below
                import traceback

                self.crash_count += 1
                self.get_logger().error(
                    f"Unhandled exception in {name} callback (#{self.crash_count}), "
                    f"node keeps running:\n{traceback.format_exc()}",
                    throttle_duration_sec=1.0,
                )
                return None

        return _wrapped

    def _grade_at(self, route_id, s_m):
        """Track grade at s, or flat ground once the estimate has drifted past the mapped route."""
        if not route_id:
            return 0.0
        try:
            return self.geometry.grade_at(route_id, s_m)
        except (KeyError, ValueError):
            self.get_logger().warning(
                f"Predicted position s={s_m:.1f} m is outside Pathgraph; using flat grade",
                throttle_duration_sec=5.0,
            )
            return 0.0

    def _next_sequence(self):
        self.sequence += 1
        return self.sequence

    def _ensure_initialized(self, stamp):
        if self.estimator.t is None:
            self.input_origin_stamp = stamp
            self.estimator.initialize(
                InitialState(
                    stamp,
                    self.s0,
                    max(0.0, float(self.get_parameter("initial_v_mps").value)),
                ),
                self.model_config,
            )
            self.estimator.route_id = self.route_id

    def _record_input_timing(self, stamp, wheel=False):
        now = self.get_clock().now().nanoseconds
        if now >= stamp:
            lateness_ms = (now - stamp) / 1e6
            self.input_lateness_ms.append(lateness_ms)
            if wheel:
                self.wheel_lateness_ms.append(lateness_ms)
            if now - stamp > self.reorder_window_ns:
                self.late_input_count += 1
        if self.estimator.t is not None and (
            stamp < self.estimator.t
            or (self.estimator.closed_time is not None and stamp <= self.estimator.closed_time)
        ):
            self.too_late_input_count += 1

    def _control(self, msg):
        stamp = ns(msg.header.stamp)
        self._record_input_timing(stamp)
        try:
            value = controller_position_to_u(msg.position)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=1.0)
            return
        self._ensure_initialized(stamp)
        queued = len(self.estimator.queue)
        self.estimator.ingest_control(ControlSample(stamp, self._next_sequence(), value))
        self._track_queued_input(stamp, queued)

    def _wheel(self, msg, wheel_id):
        stamp = ns(msg.header.stamp)
        self._record_input_timing(stamp, wheel=True)
        try:
            value = wheel_kmh_to_mps(msg.velocity)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=1.0)
            return
        self._ensure_initialized(stamp)
        queued = len(self.estimator.queue)
        self.estimator.ingest_wheel(WheelSample(stamp, self._next_sequence(), wheel_id, value))
        self._track_queued_input(stamp, queued)

    def _track_queued_input(self, stamp, previous_queue_size):
        if len(self.estimator.queue) > previous_queue_size:
            self.pending_input_timing.append((
                stamp, self.callback_received_ns or time.perf_counter_ns(),
            ))

    def _gnss_allowed(self, stamp):
        if self.gnss_policy == "disabled" or self.gnss_closed:
            return False
        if self.gnss_policy == "intermittent":
            return True
        origin = self.input_origin_stamp
        if self.gnss_initialized or (origin is not None and stamp > origin + self.gnss_window_s * 1e9):
            self.gnss_reasons.add("GNSS_INITIALIZATION_CLOSED")
            return False
        return True

    def _create_gnss_subscriptions(self):
        for receiver in ANTENNAS:
            self.gnss_subscriptions.append(
                self.create_subscription(
                    NavSatFix,
                    f"/sensing/gnss/{receiver}/fix",
                    self._guard(f"gnss fix {receiver}", lambda msg, name=receiver: self._fix(msg, name)),
                    INPUT_QOS,
                    callback_group=self._subscription_group,
                )
            )
            self.gnss_subscriptions.append(
                self.create_subscription(
                    TwistStamped,
                    f"/sensing/gnss/{receiver}/vel",
                    self._guard(
                        f"gnss velocity {receiver}",
                        lambda msg, name=receiver: self._velocity_correction(msg, name),
                    ),
                    INPUT_QOS,
                    callback_group=self._subscription_group,
                )
            )

    def _fix(self, msg, receiver):
        if not self._gnss_allowed(ns(msg.header.stamp)):
            return
        self._record_input_timing(ns(msg.header.stamp))
        values = (float(msg.latitude), float(msg.longitude), float(msg.altitude))
        if msg.status.status < 0 or not all(math.isfinite(value) for value in values):
            self.prefilter_rejected_position += 1
            self.gnss_reasons.add("GNSS_FIX_INVALID")
            return
        variance = float(msg.position_covariance[0])
        if not math.isfinite(variance) or variance <= 0:
            variance = float(self.get_parameter("gnss_position_variance_m2").value)
        self.pending_fixes[receiver] = (ns(msg.header.stamp), values, variance)
        self._pair_fixes()

    def _pair_fixes(self):
        if not all(name in self.pending_fixes for name in ANTENNAS):
            return
        master = self.pending_fixes["master"]
        rover = self.pending_fixes["rover"]
        tolerance = round(self.get_parameter("gnss_sync_tolerance_ms").value * 1e6)
        if abs(master[0] - rover[0]) > tolerance:
            older = "master" if master[0] < rover[0] else "rover"
            self._consume_single(older)
            return
        self.pending_fixes.clear()
        observation = self.geometry.observe_dual(master[1], rover[1], self.route_id)
        self._apply_observation(
            observation,
            max(master[0], rover[0]),
            max(master[2], rover[2]),
            "gnss_dual",
        )

    def _consume_single(self, receiver):
        stamp, fix, variance = self.pending_fixes.pop(receiver)
        observation = self.geometry.observe_single(fix, ANTENNAS[receiver], self.route_id)
        self._apply_observation(observation, stamp, variance * 4.0, f"gnss_{receiver}")

    def _flush_old_fixes(self, stamp):
        tolerance = round(self.get_parameter("gnss_sync_tolerance_ms").value * 1e6)
        for receiver, item in list(self.pending_fixes.items()):
            if stamp - item[0] > tolerance:
                self._consume_single(receiver)

    def _apply_observation(self, observation, stamp, variance, source):
        if not self._gnss_allowed(stamp):
            return
        if observation is None:
            self.prefilter_rejected_position += 1
            self.gnss_reasons.add("GNSS_OUT_OF_GRAPH")
            return
        if abs(observation.cross_track_m) > self.get_parameter("gnss_cross_track_gate_m").value:
            self.prefilter_rejected_position += 1
            self.gnss_reasons.add("GNSS_CROSS_TRACK_REJECTED")
            return
        if (
            observation.baseline_error_m is not None
            and abs(observation.baseline_error_m)
            > self.get_parameter("gnss_baseline_error_gate_m").value
        ):
            self.prefilter_rejected_position += 1
            self.gnss_reasons.add("GNSS_BASELINE_REJECTED")
            return
        self._ensure_initialized(stamp)
        if self.route_id is not None and observation.route_id != self.route_id:
            self.prefilter_rejected_position += 1
            self.gnss_reasons.add("GNSS_ROUTE_MISMATCH")
            return
        self.estimator.ingest_position_correction(
            AlongTrackPositionCorrection(
                stamp, self._next_sequence(), observation.route_id, observation.s_m,
                max(float(variance), 1e-6), source, initialize=self.route_id is None,
            )
        )
        self.last_gnss_stamp = stamp

    def _disable_gnss(self):
        # Humble's multithreaded executor may already have a subscription in its wait set.
        # Destroying it from the timer callback races _take_subscription (outside our guard)
        # and crashes with InvalidHandle. Keep handles alive until node teardown, but close
        # the input gate permanently so queued/subsequent callbacks cannot apply corrections.
        self.gnss_closed = True
        self.pending_fixes.clear()
        self.gnss_reasons.add("GNSS_INITIALIZATION_CLOSED")

    def _velocity_correction(self, msg, receiver):
        del receiver
        stamp = ns(msg.header.stamp)
        if not self._gnss_allowed(stamp):
            return
        self._record_input_timing(stamp)
        if self.estimator.route_id is None or self.estimator.t is None:
            self.prefilter_rejected_velocity += 1
            return
        try:
            pose = self.geometry.body_pose_at(self.route_id, self.estimator.x[0])
        except (KeyError, ValueError) as exc:
            self.prefilter_rejected_velocity += 1
            self.gnss_reasons.add("OUT_OF_GRAPH")
            self.get_logger().warning(f"GNSS velocity correction outside Pathgraph: {exc}",
                                       throttle_duration_sec=1.0)
            return
        linear = msg.twist.linear
        velocity = float(linear.x) * math.cos(pose.yaw_rad) + float(linear.y) * math.sin(
            pose.yaw_rad
        )
        if not math.isfinite(velocity) or velocity < 0:
            self.prefilter_rejected_velocity += 1
            self.gnss_reasons.add("GNSS_VELOCITY_INVALID")
            return
        self.estimator.ingest_velocity_correction(
            LongitudinalVelocityCorrection(
                stamp,
                self._next_sequence(),
                velocity,
                float(self.get_parameter("gnss_velocity_variance_m2ps2").value),
                "gnss_velocity",
            )
        )
        self.last_gnss_stamp = stamp

    def _tick(self):
        if self.estimator.t is None:
            return
        stamp = self.get_clock().now().nanoseconds
        if stamp < self.estimator.t:
            return
        self._flush_old_fixes(stamp)
        if (
            self.gnss_policy == "initialization_only" and not self.gnss_closed
            and stamp > self.input_origin_stamp + self.gnss_window_s * 1e9
        ):
            self._disable_gnss()
            self.gnss_reasons.add("GNSS_INITIALIZATION_CLOSED")
        if stamp == self.last_publish_stamp:
            return
        began = time.perf_counter()
        result, self.filter_commit_stamp = advance_fixed_lag(
            self.estimator, stamp, self.reorder_window_ns
        )
        compute_ms = (time.perf_counter() - began) * 1000.0
        self._publish(result, stamp, compute_ms)

    def _publish(self, result, stamp, compute_ms):
        if result.route_id:
            self.route_id = result.route_id
        result.rejected_gnss_position_count += self.prefilter_rejected_position
        result.rejected_gnss_velocity_count += self.prefilter_rejected_velocity
        result.reason_codes.extend(sorted(self.gnss_reasons))
        self.gnss_reasons.clear()
        if self.gnss_policy == "initialization_only" and not self.gnss_closed and (
            self.gnss_initialized or result.accepted_gnss_position_count > 0
        ):
            self.gnss_initialized = True
            self._disable_gnss()
        wall_now, cpu_now = time.perf_counter(), time.process_time()
        wall_delta = wall_now - self.last_perf_wall
        if wall_delta > 0:
            self.cpu_cores = max(0.0, (cpu_now - self.last_perf_cpu) / wall_delta)
        self.last_perf_wall, self.last_perf_cpu = wall_now, cpu_now
        map_pose = None
        if result.s_m is not None and self.route_id:
            try:
                map_pose = self.geometry.body_pose_at(self.route_id, result.s_m)
            except (KeyError, ValueError):
                result.reason_codes.append("OUT_OF_GRAPH")
        elif self.route_id is None:
            result.reason_codes.append("RELATIVE_POSITION_ONLY")
        gnss_age = None if self.last_gnss_stamp is None else (stamp - self.last_gnss_stamp) / 1e9
        estimate = estimate_message(
            result,
            self.get_parameter("run_id").value,
            compute_ms,
            map_pose,
            self.gnss_policy,
            gnss_age,
        )
        self.estimate_publisher.publish(estimate)
        if result.v_mps is not None:
            velocity = VelocitySensor()
            set_stamp(velocity.header, stamp)
            velocity.header.frame_id = "base_link"
            velocity.velocity = result.v_mps
            self.velocity_publisher.publish(velocity)
        # Keep both outputs continuous during degradation; the frame and diagnostics distinguish
        # mapped XY from one-dimensional odometry. Validity is carried by /odometry/estimate.
        if result.s_m is not None and result.covariance_4x4 is not None:
            self.position_publisher.publish(self._odometry(result, map_pose, stamp))
        published_ns = time.perf_counter_ns()
        pending, samples = deque(maxlen=4096), []
        for input_stamp, received_ns in self.pending_input_timing:
            if input_stamp <= stamp:
                samples.append(max(0.0, (published_ns - received_ns) / 1e6))
            else:
                pending.append((input_stamp, received_ns))
        self.pending_input_timing = pending
        self.latency_ms.extend(samples)
        self.latest_input_to_publication_ms = max(samples, default=None)
        self._publish_diagnostics(result, compute_ms, stamp, map_pose is not None)
        self.last_publish_stamp = stamp

    def _odometry(self, result, pose, stamp):
        message = Odometry()
        set_stamp(message.header, stamp)
        message.header.frame_id = "map"
        message.child_frame_id = "base_link"
        if pose is None:
            # One-dimensional track coordinates, not a fabricated XY trajectory. Without a
            # route the origin is the configured start; beyond a route use absolute along-track s.
            message.header.frame_id = f"track/{self.route_id}" if self.route_id else "odom"
            message.pose.pose.position.x = result.s_m if self.route_id else result.s_m - self.s0
            message.pose.pose.orientation.w = 1.0
            message.pose.covariance[0] = max(result.covariance_4x4[0], 1e-12)
            message.twist.twist.linear.x = result.v_mps
            message.twist.covariance[0] = max(result.covariance_4x4[5], 1e-12)
            for index in (7, 14, 21, 28, 35):
                message.pose.covariance[index] = message.twist.covariance[index] = 1e6
            return message
        message.pose.pose.position.x = pose.x_m
        message.pose.pose.position.y = pose.y_m
        message.pose.pose.position.z = pose.z_m
        message.pose.pose.orientation.z = math.sin(pose.yaw_rad / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw_rad / 2.0)
        message.twist.twist.linear.x = result.v_mps
        along = max(result.covariance_4x4[0], 1e-12)
        cross = float(self.get_parameter("cross_track_variance_m2").value)
        cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
        message.pose.covariance[0] = cosine * cosine * along + sine * sine * cross
        message.pose.covariance[7] = sine * sine * along + cosine * cosine * cross
        message.pose.covariance[1] = message.pose.covariance[6] = cosine * sine * (along - cross)
        message.pose.covariance[14] = 25.0
        message.pose.covariance[21] = 1e6
        message.pose.covariance[28] = 1e6
        message.pose.covariance[35] = float(self.get_parameter("yaw_variance_rad2").value)
        message.twist.covariance[0] = max(result.covariance_4x4[5], 1e-12)
        for index in (7, 14, 21, 28, 35):
            message.twist.covariance[index] = 1e6
        return message

    def _publish_diagnostics(self, result, compute_ms, stamp, has_map_pose):
        array = DiagnosticArray()
        set_stamp(array.header, stamp)
        status = DiagnosticStatus()
        status.name = "reserve_odometry"
        status.hardware_id = self.vehicle_id
        status.level = DiagnosticStatus.OK if result.valid else DiagnosticStatus.ERROR
        if (result.mode != "FUSED" or not has_map_pose) and result.valid:
            status.level = DiagnosticStatus.WARN
        status.message = result.mode
        input_to_publication_ms = self.latest_input_to_publication_ms
        rss_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        latency_p95 = percentile(self.latency_ms, 0.95)
        latency_p99 = percentile(self.latency_ms, 0.99)
        input_p95 = percentile(self.input_lateness_ms, 0.95)
        input_p99 = percentile(self.input_lateness_ms, 0.99)
        wheel_p95 = percentile(self.wheel_lateness_ms, 0.95)
        wheel_p99 = percentile(self.wheel_lateness_ms, 0.99)
        commit_lag_ms = (
            None if self.filter_commit_stamp is None else (stamp - self.filter_commit_stamp) / 1e6
        )
        status.values = [
            KeyValue(key="reasons", value=",".join(result.reason_codes)),
            KeyValue(key="compute_ms", value=f"{compute_ms:.6f}"),
            KeyValue(key="route_id", value=self.route_id or ""),
            KeyValue(key="gnss_policy", value=self.gnss_policy),
            KeyValue(key="has_map_pose", value=str(has_map_pose).lower()),
            KeyValue(key="queue_depth", value=str(len(self.estimator.queue))),
            KeyValue(key="reorder_window_ms", value=f"{self.reorder_window_ns / 1e6:.3f}"),
            KeyValue(key="filter_commit_lag_ms", value=f"{commit_lag_ms or 0.0:.3f}"),
            KeyValue(key="input_lateness_p95_ms", value=f"{input_p95 or 0.0:.3f}"),
            KeyValue(key="input_lateness_p99_ms", value=f"{input_p99 or 0.0:.3f}"),
            KeyValue(key="input_lateness_max_ms", value=f"{max(self.input_lateness_ms, default=0.0):.3f}"),
            KeyValue(key="wheel_lateness_p95_ms", value=f"{wheel_p95 or 0.0:.3f}"),
            KeyValue(key="wheel_lateness_p99_ms", value=f"{wheel_p99 or 0.0:.3f}"),
            KeyValue(key="late_input_count", value=str(self.late_input_count)),
            KeyValue(key="too_late_input_count", value=str(self.too_late_input_count)),
            KeyValue(key="crash_count", value=str(self.crash_count)),
            KeyValue(key="latency_definition", value="wheel/control callback entry to result publication; steady clock"),
            KeyValue(key="input_to_publication_ms", value=str(input_to_publication_ms)),
            KeyValue(key="latency_samples", value=str(len(self.latency_ms))),
            KeyValue(key="latency_p95_ms", value=str(latency_p95)),
            KeyValue(key="latency_p99_ms", value=str(latency_p99)),
            KeyValue(key="latency_max_ms", value=str(max(self.latency_ms, default=None))),
            KeyValue(key="cpu_cores", value=f"{self.cpu_cores:.6f}"),
            KeyValue(key="rss_mb", value=f"{rss_mb:.3f}"),
        ]
        array.status = [status]
        self.diagnostics_publisher.publish(array)


def main():
    rclpy.init()
    node = ReserveOdometryNode()
    # A single-threaded spin serialized message reception behind the 50 Hz tick; a matched but tiny
    # organizer publisher history depth then silently dropped whatever arrived while the tick was
    # busy. Two threads are enough: one drains subscriptions (their own reentrant group), the other
    # runs the tick (its own group) — see the callback-group comment in __init__ for why that pairing
    # is what actually needs the concurrency, and self._lock in _guard for how they stay safe together.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except Exception:
        import traceback

        node.get_logger().fatal(f"reserve_odometry_node is exiting unexpectedly:\n{traceback.format_exc()}")
        raise
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
