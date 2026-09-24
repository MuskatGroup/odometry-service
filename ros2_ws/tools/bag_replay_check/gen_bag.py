import json
import rosbag2_py
from rclpy.serialization import serialize_message
from odometry_msgs.msg import ControlSample, WheelSample
from odometry_io.emulator import simulate

T0 = 100_000_000_000  # sim time starts at 100 s, unrelated to wall clock
config = {"seed": 17, "steps": 400, "step_ns": "50000000", "channels": 1, "initial_speed_mps": 5,
          "noise_mps": 0.02, "dropout_steps": [120, 150], "slip_steps": [220, 240]}

writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri="/tmp/bag", storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
writer.create_topic(rosbag2_py.TopicMetadata(name="/tram/control", type="odometry_msgs/msg/ControlSample", serialization_format="cdr"))
writer.create_topic(rosbag2_py.TopicMetadata(name="/tram/wheel_speed", type="odometry_msgs/msg/WheelSample", serialization_format="cdr"))


def stamp(msg, ns):
    msg.header.stamp.sec, msg.header.stamp.nanosec = ns // 1_000_000_000, ns % 1_000_000_000
    msg.header.frame_id = "tram"


truth = open("/tmp/truth.jsonl", "w")
for i, (rec, ref) in enumerate(simulate(config)):
    ns = T0 + int(rec["stamp_ns"])
    c = ControlSample(); stamp(c, ns); c.seq = i; c.u = float(rec["control"]); c.valid = True
    writer.write("/tram/control", serialize_message(c), ns)
    if "0" in rec["speeds"]:  # dropout = no message, not zero
        w = WheelSample(); stamp(w, ns); w.seq = i; w.wheel_id = "w"; w.speed_mps = float(rec["speeds"]["0"]); w.valid = True
        writer.write("/tram/wheel_speed", serialize_message(w), ns)
    truth.write(json.dumps({"stamp_ns": ns, "v": ref["v_mps"], "s": ref["s_m"], "wheel": rec["speeds"].get("0")}) + "\n")
truth.close()
print("bag written")
