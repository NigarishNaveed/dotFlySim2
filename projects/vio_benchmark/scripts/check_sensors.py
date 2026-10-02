#!/usr/bin/env python3
"""
check_sensors.py -- check what OpenVINS is fed, from a bag made by
record_sensor_check.sh (aircraft on the ground). Each check prints PASS/FAIL
and what the number should be.

    python3 check_sensors.py ~/bags/vio_sensorcheck_<time>

Runs in the container (rosbag2_py) or anywhere with `pip install mcap
mcap-ros2-support numpy opencv-python-headless`.

  1. IMU frame       at rest, linear_acceleration should be ~(0, 0, +9.8):
                     FLU, which is what kalibr_imu_chain.yaml says
  2. IMU rate/stamps ~50 Hz, monotonic, no big gaps
  3. one clock       IMU, image and ground-truth stamps overlap in time
  4. stereo pairs    left and right stamps match
  5. right way up    sky (brighter than the ground) at the TOP of the image
  6. left is left    for the pair in kalibr_imucam_chain.yaml (cam0 at +y),
                     a point appears further RIGHT in the left image:
                     x_left - x_right > 0, and rows agree (rectified)
  7. texture         how many trackable corners a frame has, and how many of
                     them are on the GROUND (the only near features)
"""

import json
import math
import sys

import numpy as np

IMU = "/wrapper/psdk_ros2/imu"
LEFT = "/wrapper/psdk_ros2/perception_stereo_left_stream"
RIGHT = "/wrapper/psdk_ros2/perception_stereo_right_stream"
GT = "/drone/ground_truth/odom"
RAW_LEFT = "/drone/perception/front/left/image_raw"     # straight out of Gazebo,
RAW_RIGHT = "/drone/perception/front/right/image_raw"   # before the PSDK bridge


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def image_array(msg):
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return data.reshape(msg.height, msg.step)[:, : msg.width]


def read(uri):
    """-> dict topic -> list of (recv_time_s, msg)"""
    out = {IMU: [], LEFT: [], RIGHT: [], GT: [], RAW_LEFT: [], RAW_RIGHT: []}
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                    rosbag2_py.ConverterOptions("cdr", "cdr"))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic in out:
                out[topic].append((t * 1e-9, deserialize_message(data, types[topic])))
        return out
    except ImportError:
        pass
    import glob
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    for path in sorted(glob.glob(uri.rstrip("/") + "/*.mcap")):
        with open(path, "rb") as f:
            for _, ch, msg, dec in make_reader(f, decoder_factories=[DecoderFactory()]).iter_decoded_messages():
                if ch.topic in out:
                    out[ch.topic].append((msg.log_time * 1e-9, dec))
    return out


def verdict(ok, text):
    ok = bool(ok)  # numpy bools are not JSON-serialisable
    print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
    return ok


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = read(sys.argv[1])
    print({k: len(v) for k, v in d.items()})
    results = {}

    # 1-2 IMU
    imu = d[IMU]
    if imu:
        acc = np.array([[m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] for _, m in imu])
        gyr = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z] for _, m in imu])
        ts = np.array([stamp(m.header) for _, m in imu])
        mean_acc = acc.mean(axis=0)
        print("\n1. IMU at rest")
        print(f"     accel mean {np.round(mean_acc, 3)}  std {np.round(acc.std(axis=0), 3)}  |a| {np.linalg.norm(mean_acc):.3f}")
        print(f"     gyro  mean {np.round(gyr.mean(axis=0), 4)}  std {np.round(gyr.std(axis=0), 4)}")
        results["imu_flu"] = verdict(mean_acc[2] > 9.0 and abs(mean_acc[0]) < 1.0 and abs(mean_acc[1]) < 1.0,
                                     "gravity +9.8 on z (FLU). Negative z = FRD; on x/y = axes swapped")
        dt = np.diff(ts)
        print("\n2. IMU timing")
        print(f"     {len(ts) / max(ts[-1] - ts[0], 1e-9):.1f} Hz over stamps, dt median {np.median(dt)*1e3:.1f} ms, "
              f"min {dt.min()*1e3:.1f}, max {dt.max()*1e3:.1f}, non-increasing {int(np.sum(dt <= 0))}")
        results["imu_rate"] = verdict(40 < len(ts) / max(ts[-1] - ts[0], 1e-9) < 60 and np.sum(dt <= 0) == 0
                                      and dt.max() < 0.1, "~50 Hz, monotonic, no gap > 100 ms")
    else:
        print("no IMU messages")

    # 3 clocks
    lefts, rights, gts = d[LEFT], d[RIGHT], d[GT]
    print("\n3. clocks (header stamps, s)")
    for name, rows in (("imu", imu), ("left", lefts), ("right", rights), ("truth", gts)):
        if rows:
            s = [stamp(m.header) for _, m in rows]
            r = [t for t, _ in rows]
            print(f"     {name:6s} {min(s):12.3f} .. {max(s):12.3f}   stamp - receive: {np.median(np.array(s) - np.array(r)):+.3f} s")
    if imu and lefts:
        si = np.array([stamp(m.header) for _, m in imu]); sl = np.array([stamp(m.header) for _, m in lefts])
        results["one_clock"] = verdict(abs(np.median(si) - np.median(sl)) < 5.0, "IMU and images stamped on the same clock")
        # IMU latency relative to images: both against their receive times
        ri = np.array([t for t, _ in imu]); rl = np.array([t for t, _ in lefts])
        print(f"     receive minus stamp: imu {np.median(ri - si)*1e3:+.0f} ms, images {np.median(rl - sl)*1e3:+.0f} ms "
              "(the difference is roughly the camera-IMU time offset)")

    if not (lefts and rights):
        print("no stereo images")
        return 1

    # 4 pairing
    lt = {round(stamp(m.header), 3): m for _, m in lefts}
    rt = {round(stamp(m.header), 3): m for _, m in rights}
    common = sorted(set(lt) & set(rt))
    print(f"\n4. stereo pairs: {len(lefts)} left, {len(rights)} right, {len(common)} with identical stamps")
    results["pairs"] = verdict(len(common) > 0.8 * min(len(lefts), len(rights)), "left/right stamps match")
    if not common:
        return 1
    L = image_array(lt[common[len(common) // 2]]); R = image_array(rt[common[len(common) // 2]])
    print(f"     image {L.shape[1]}x{L.shape[0]} encoding {lt[common[0]].encoding}")

    # 4b where do left and right become the same picture?
    def identical(left_rows, right_rows):
        a = {round(stamp(m.header), 3): m for _, m in left_rows}
        b = {round(stamp(m.header), 3): m for _, m in right_rows}
        both = sorted(set(a) & set(b))
        same = sum(bytes(a[k].data) == bytes(b[k].data) for k in both)
        return same, len(both)
    same, n = identical(lefts, rights)
    results["wrapper_pair_differs"] = verdict(same < n, f"wrapper streams: {same} of {n} left/right pairs byte-identical "
                                              "(two cameras 9 cm apart can never be)")
    if d[RAW_LEFT] and d[RAW_RIGHT]:
        same, n = identical(d[RAW_LEFT], d[RAW_RIGHT])
        results["gazebo_pair_differs"] = verdict(same < n, f"Gazebo topics (before the bridge): {same} of {n} pairs byte-identical")
        if results["gazebo_pair_differs"] and not results["wrapper_pair_differs"]:
            print("     -> the cameras differ out of Gazebo and become identical in the PSDK bridge")
        elif not results["gazebo_pair_differs"]:
            print("     -> already identical out of Gazebo / ros_gz_bridge: the bridge only passes it on")

    # 5 orientation
    h = L.shape[0]
    top, bottom = L[: h // 4], L[3 * h // 4:]
    print(f"\n5. top quarter mean {top.mean():.0f} (std {top.std():.0f}), bottom quarter mean {bottom.mean():.0f} (std {bottom.std():.0f})")
    # Brightness, not texture: in this world the clouds carry more texture than
    # the ground, which made a texture test call an upright image upside down.
    results["upright"] = verdict(top.mean() > bottom.mean(), "brighter sky at the top, darker ground at the bottom")

    # 6-7 stereo geometry and texture
    try:
        import cv2
    except ImportError:
        print("\n(no OpenCV -- skipping the stereo checks)")
        return 0
    orb = cv2.ORB_create(2000)
    kl, dl = orb.detectAndCompute(L, None)
    kr, dr = orb.detectAndCompute(R, None)
    fast = cv2.FastFeatureDetector_create(15).detect(L)
    print(f"\n7. texture: {len(fast)} FAST corners (threshold 15, as OpenVINS), {len(kl)} ORB")
    results["texture"] = verdict(len(fast) > 100, "enough corners to track (OpenVINS wants ~200 per camera)")
    # Corners on the GROUND, below the horizon (image row 352 when level). Sky
    # and clouds are effectively at infinity -- they tell an estimator how it
    # turns, never how far it moved -- so these are the ones that matter. The
    # flat-colour ground gave exactly 0 (dotFlySim); models/ground_scrub fixes it.
    ground = [k for k in fast if k.pt[1] > 380]
    print(f"   of which on the ground (below the horizon): {len(ground)}")
    results["ground_texture"] = verdict(len(ground) > 50, "trackable corners on the ground, not only sky and towers")
    if dl is None or dr is None:
        print("   no descriptors -- nothing to match")
        return 0
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(dl, dr)
    if len(matches) < 10:
        print(f"   only {len(matches)} matches -- cannot judge stereo geometry")
        return 0
    dx = np.array([kl[m.queryIdx].pt[0] - kr[m.trainIdx].pt[0] for m in matches])
    dy = np.array([kl[m.queryIdx].pt[1] - kr[m.trainIdx].pt[1] for m in matches])
    good = np.abs(dy) < 3.0
    print(f"\n6. stereo: {len(matches)} matches, {int(good.sum())} on (nearly) the same row")
    print(f"     row difference median {np.median(dy):+.2f} px; x_left - x_right median {np.median(dx[good]):+.2f} px, "
          f"{100 * np.mean(dx[good] > 0):.0f} % positive")
    results["rectified"] = verdict(good.mean() > 0.5, "rows agree (rectified pair)")
    results["left_is_left"] = verdict(np.mean(dx[good] > 0) > 0.7,
                                      "x_left - x_right > 0: the LEFT stream is the camera at +y. "
                                      "Mostly negative = left/right swapped")
    print("\nsummary:", json.dumps(results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
