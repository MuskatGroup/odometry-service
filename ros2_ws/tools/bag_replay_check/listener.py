import json
import rclpy
from rclpy.node import Node
from odometry_msgs.msg import LongitudinalEstimate

MODES = ["INITIALIZING", "FUSED", "DEGRADED", "MODEL_ONLY", "INVALID"]


class Listener(Node):
    def __init__(self):
        super().__init__("listener", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.out = open("/tmp/est.jsonl", "w")
        self.create_subscription(LongitudinalEstimate, "/odometry/estimate", self.cb, 50)

    def cb(self, m):
        ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        self.out.write(json.dumps({"stamp_ns": ns, "mode": MODES[m.mode], "valid": m.valid, "has": m.has_estimate,
                                   "v": m.v_mps, "s": m.s_m, "sigma_s": m.sigma_s_m, "reasons": list(m.reason_codes)}) + "\n")
        self.out.flush()


rclpy.init()
rclpy.spin(Listener())
