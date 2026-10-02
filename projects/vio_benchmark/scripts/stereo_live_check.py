#!/usr/bin/env python3
"""
stereo_live_check.py -- refuse to fly with a broken stereo pair. Runs in the
container: waits for a left and a right frame with the same stamp and exits 0
if they differ, 1 if they are byte-identical, 2 if no pair arrives in time.

Why: in one simulator session both perception streams carried the SAME
picture (146 of 146 pairs identical, and tower_1 missing from it). A
visual-inertial estimator fed that sees zero parallax everywhere and diverges
within seconds. A restart cleared it, so it is checked before every flight
rather than assumed away.
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PREFIX = "/wrapper/psdk_ros2/perception_stereo_"


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("stereo_live_check")
    frames = {"left": {}, "right": {}}

    def keep(side):
        def cb(msg):
            key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            frames[side][key] = bytes(msg.data)
        return cb

    node.create_subscription(Image, PREFIX + "left_stream", keep("left"), qos_profile_sensor_data)
    node.create_subscription(Image, PREFIX + "right_stream", keep("right"), qos_profile_sensor_data)
    deadline = time.monotonic() + float(sys.argv[1] if len(sys.argv) > 1 else 20.0)
    verdict = 2
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        common = set(frames["left"]) & set(frames["right"])
        if len(common) >= 3:
            same = sum(frames["left"][k] == frames["right"][k] for k in common)
            print(f"stereo pair: {same} of {len(common)} left/right frames byte-identical")
            verdict = 1 if same == len(common) else 0
            break
    if verdict == 2:
        print(f"no stereo pair arrived: left {len(frames['left'])}, right {len(frames['right'])} frames")
    node.destroy_node()
    rclpy.shutdown()
    return verdict


if __name__ == "__main__":
    sys.exit(main())
