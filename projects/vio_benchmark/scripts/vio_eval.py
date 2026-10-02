#!/usr/bin/env python3
"""
vio_eval.py -- score an estimator's trajectory against Gazebo ground truth,
over the SEGMENT that demo_vio_benchmark flew (a helix or a transit).

Runs inside the container (it needs rosbag2_py), on one bag or several:

    python3 vio_eval.py ~/bags/vio_helix_2026-09-30_16-00-00
    python3 vio_eval.py ~/bags/vio_transit_*  --plot

scripts/vio_trial.sh calls it for you after each trial.

WHAT IS READ
    /ov_msckf/poseimu          the estimate (PoseWithCovarianceStamped)
    /drone/ground_truth/odom   the truth, 100 Hz, world ENU (models/m4e/model.sdf)
    /vio_benchmark/phase       JSON per tick: sim time, step, segment geometry
Every stamp here is simulation time, so they line up directly.

WHAT IS REPORTED, and what each number means
    ATE (4-DOF)   One best-fit shift and yaw rotation of the whole estimate onto
                  the truth, then the RMS position error. The standard VIO
                  score: roll, pitch and scale are observable to a VIO, so
                  they are NOT aligned away; yaw and position are not.
    drift         The estimate lined up with the truth at the START of the
                  segment only (first pose, position and yaw), then left
                  alone. This is what an aircraft flying on the estimate would
                  experience. Reported at the end, as % of distance flown, and
                  split into components that matter for each path:
                    helix    per completed turn, and vertical vs horizontal --
                             vertical drift eats directly into the view overlap
                    transit  per span (at each tower), and along-track (scale)
                             vs cross-track vs vertical
    RPE           Error over every sub-stretch of a given length, each one
                  lined up at its own start. Tells local accuracy apart from
                  accumulated drift.
    scale         Estimate length / true length, from a similarity (7-DOF)
                  fit. 1.000 is perfect, 1.02 is 2 % too long. A transit with
                  weak parallax shows it here first.
    health        estimate rate, the longest gap, and jumps (> 1 m between
                  consecutive estimates) -- a filter that reset mid-segment
                  shows up here, and its drift numbers mean little.

OUTPUT, per bag, in <bag>/eval/:
    summary.json   every number above
    errors.csv     per-sample drift error and its components
    est_tum.txt, gt_tum.txt   the segment in TUM format, for ov_eval or evo:
                   ros2 run ov_eval error_singlerun posyaw gt_tum.txt est_tum.txt
    trajectory.png with --plot: the WHOLE flight, ground truth and OpenVINS
                   on the same axes -- top view, side view, and the deviation
                   between them (plot_same_graph.py)
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

EST_TOPIC = "/ov_msckf/poseimu"
GT_TOPIC = "/drone/ground_truth/odom"
PHASE_TOPIC = "/vio_benchmark/phase"

TOWERS_EAST = [60.0, 260.0, 460.0, 660.0, 860.0]   # worlds/README.md


# ─────────────────────────────────────────────────────────────────────────────
# reading
# ─────────────────────────────────────────────────────────────────────────────

def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def read_bag(uri, est_topic=EST_TOPIC, gt_topic=GT_TOPIC, phase_topic=PHASE_TOPIC):
    """-> est, gt: arrays of [t, x, y, z, qx, qy, qz, qw]; phases: list of dicts."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (est_topic, gt_topic, phase_topic):
        if need not in types:
            raise RuntimeError(f"{uri}: no {need} in the bag (has: {', '.join(sorted(types))})")
    reader.set_filter(rosbag2_py.StorageFilter(topics=[est_topic, gt_topic, phase_topic]))
    classes = {name: get_message(types[name]) for name in (est_topic, gt_topic, phase_topic)}

    est, gt, phases = [], [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        if topic == phase_topic:
            try:
                phases.append(json.loads(msg.data))
            except ValueError:
                pass
            continue
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose            # PoseWithCovariance and Odometry alike
        row = [stamp, pose.position.x, pose.position.y, pose.position.z,
               pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        (est if topic == est_topic else gt).append(row)

    def as_array(rows):
        a = np.array(rows, dtype=float).reshape(-1, 8)
        return a[np.argsort(a[:, 0], kind="stable")]

    return as_array(est), as_array(gt), phases


# ─────────────────────────────────────────────────────────────────────────────
# geometry
# ─────────────────────────────────────────────────────────────────────────────

def wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def rot_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def align_posyaw(E, G):
    """Best yaw + translation taking E onto G (least squares). -> R, t."""
    me, mg = E.mean(axis=0), G.mean(axis=0)
    Ec, Gc = E - me, G - mg
    yaw = math.atan2(np.sum(Ec[:, 0] * Gc[:, 1] - Ec[:, 1] * Gc[:, 0]),
                     np.sum(Ec[:, 0] * Gc[:, 0] + Ec[:, 1] * Gc[:, 1]))
    R = rot_z(yaw)
    return R, mg - R @ me


def umeyama_scale(E, G):
    """How big the estimate is against the truth: 1 / the scale of the best
    similarity transform E -> G (Umeyama 1991). 1.02 = the estimate is 2 %
    too long, 0.98 = 2 % too short."""
    me, mg = E.mean(axis=0), G.mean(axis=0)
    Ec, Gc = E - me, G - mg
    var_e = np.mean(np.sum(Ec * Ec, axis=1))
    if var_e < 1e-12:
        return float("nan")
    U, D, Vt = np.linalg.svd(Gc.T @ Ec / len(E))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1.0
    s = float(np.trace(np.diag(D) @ S) / var_e)
    return 1.0 / s if s > 0.0 else float("nan")


def stats(x):
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), "rmse": float(np.sqrt(np.mean(x * x))), "mean": float(np.mean(x)),
            "median": float(np.median(x)), "max": float(np.max(x))}


# ─────────────────────────────────────────────────────────────────────────────
# the segment
# ─────────────────────────────────────────────────────────────────────────────

def segment_window(phases, name="SEGMENT"):
    rows = [p for p in phases if p.get("phase") == name]
    if not rows:
        seen = sorted({p.get("phase") for p in phases})
        raise RuntimeError(f"no {name} phase in the bag (phases seen: {seen}) -- "
                           "the flight never reached the measured part")
    return rows[0]["t"], rows[-1]["t"], rows[-1]


def associate(est, gt, t0, t1, max_gap=0.05):
    """Ground truth interpolated at each estimate stamp inside [t0, t1]."""
    inside = est[(est[:, 0] >= t0) & (est[:, 0] <= t1)]
    if len(inside) < 10:
        raise RuntimeError(f"only {len(inside)} estimates inside the segment -- was the estimator "
                           "running and initialised?")
    gt_t = gt[:, 0]
    idx = np.searchsorted(gt_t, inside[:, 0])
    ok = (idx > 0) & (idx < len(gt_t))
    idx_c = np.clip(idx, 1, len(gt_t) - 1)
    gap = gt_t[idx_c] - gt_t[idx_c - 1]
    ok &= gap <= max_gap
    inside, idx_c = inside[ok], idx_c[ok]

    t = inside[:, 0]
    G = np.column_stack([np.interp(t, gt_t, gt[:, k]) for k in (1, 2, 3)])
    gyaw_all = np.unwrap([yaw_of(*q) for q in gt[:, 4:8]])
    gyaw = wrap(np.interp(t, gt_t, gyaw_all))
    E = inside[:, 1:4]
    eyaw = np.array([yaw_of(*q) for q in inside[:, 4:8]])
    return t, E, eyaw, G, gyaw


def rpe(t, E, eyaw, G, gyaw, dist, lengths, stride_s=0.5):
    """Relative error over sub-stretches of each length (metres of true path)."""
    out = {}
    starts = [0]
    for i in range(1, len(t)):
        if t[i] - t[starts[-1]] >= stride_s:
            starts.append(i)
    for d in lengths:
        trans, yaw_err = [], []
        for i in starts:
            j = int(np.searchsorted(dist, dist[i] + d))
            if j >= len(t):
                break
            dG = rot_z(-gyaw[i]) @ (G[j] - G[i])
            dE = rot_z(-eyaw[i]) @ (E[j] - E[i])
            trans.append(np.linalg.norm(dE - dG))
            yaw_err.append(abs(wrap((eyaw[j] - eyaw[i]) - (gyaw[j] - gyaw[i]))))
        if trans:
            s = stats(trans)
            s["percent_of_length"] = 100.0 * s["mean"] / d
            s["yaw_mean_deg"] = float(np.degrees(np.mean(yaw_err)))
            out[f"{d:g}m"] = s
    return out


def evaluate(est, gt, phases):
    t0, t1, info = segment_window(phases)
    profile = info.get("profile", "?")
    t, E, eyaw, G, gyaw = associate(est, gt, t0, t1)

    step = np.linalg.norm(np.diff(G, axis=0), axis=1)
    dist = np.concatenate([[0.0], np.cumsum(step)])
    length = float(dist[-1])
    duration = float(t[-1] - t[0])

    result = {"profile": profile, "segment": {"t0": t0, "t1": t1, "duration_s": duration,
              "length_m": length, "samples": int(len(t))}, "config": info}

    # health: gaps and jumps in the estimate itself
    est_in = est[(est[:, 0] >= t0) & (est[:, 0] <= t1)]
    gaps = np.diff(est_in[:, 0]) if len(est_in) > 1 else np.array([0.0])
    jumps = np.linalg.norm(np.diff(est_in[:, 1:4], axis=0), axis=1) if len(est_in) > 1 else []
    result["health"] = {
        "rate_hz": float(len(est_in) / max(duration, 1e-9)),
        "longest_gap_s": float(np.max(gaps)),
        "jumps_over_1m": int(np.sum(np.asarray(jumps) > 1.0)),
        "started_late_s": float(max(0.0, est_in[0, 0] - t0)),
    }

    # ATE, 4-DOF
    R, tr = align_posyaw(E, G)
    Ea = E @ R.T + tr
    result["ate_posyaw"] = stats(np.linalg.norm(Ea - G, axis=1))
    result["ate_posyaw"]["yaw_rmse_deg"] = float(np.degrees(np.sqrt(np.mean(
        wrap(eyaw + math.atan2(R[1, 0], R[0, 0]) - gyaw) ** 2))))
    result["scale_sim3"] = umeyama_scale(E, G)

    # drift: lined up on the first pose only
    R0 = rot_z(gyaw[0] - eyaw[0])
    Ed = (E - E[0]) @ R0.T + G[0]
    err = Ed - G
    norm = np.linalg.norm(err, axis=1)
    yaw_drift = wrap(eyaw + (gyaw[0] - eyaw[0]) - gyaw)
    drift = {
        "final_m": float(norm[-1]), "max_m": float(np.max(norm)),
        "final_percent_of_distance": float(100.0 * norm[-1] / max(length, 1e-9)),
        "final_vertical_m": float(err[-1, 2]),
        "final_horizontal_m": float(np.linalg.norm(err[-1, :2])),
        "final_yaw_deg": float(np.degrees(yaw_drift[-1])),
    }
    columns = {"t": t - t[0], "dist": dist, "err": norm, "ex": err[:, 0], "ey": err[:, 1],
               "ez": err[:, 2], "yaw_err_deg": np.degrees(yaw_drift)}

    if profile == "helix":
        c = np.array(info["centre"], dtype=float)
        angle = np.unwrap(np.arctan2(G[:, 1] - c[1], G[:, 0] - c[0]))
        turns = np.abs(angle - angle[0]) / (2.0 * np.pi)
        per_turn = []
        for k in range(1, int(turns[-1]) + 1):
            i = int(np.searchsorted(turns, k))
            if i < len(t):
                radial_dir = G[i, :2] - c
                radial_dir /= max(np.linalg.norm(radial_dir), 1e-9)
                per_turn.append({"turn": k, "t": float(t[i] - t[0]), "dist_m": float(dist[i]),
                                 "error_m": float(norm[i]), "vertical_m": float(err[i, 2]),
                                 "radial_m": float(err[i, :2] @ radial_dir),
                                 "yaw_deg": float(np.degrees(yaw_drift[i]))})
        drift["turns_flown"] = float(turns[-1])
        drift["per_turn"] = per_turn
        pitch = float(info.get("pitch", float("nan")))
        # How much of the planned overlap survives the vertical drift: a
        # vertical error of dz shifts turn k's view by dz against turn k-1's.
        if per_turn and pitch == pitch:
            dz = [abs(per_turn[0]["vertical_m"])] + [
                abs(per_turn[k]["vertical_m"] - per_turn[k - 1]["vertical_m"])
                for k in range(1, len(per_turn))]
            overlap = float(info.get("overlap", 0.0))
            footprint = pitch / max(1.0 - overlap, 1e-9)
            drift["worst_turn_to_turn_vertical_m"] = float(max(dz))
            drift["overlap_planned"] = overlap
            drift["overlap_worst_case"] = float(1.0 - (pitch + max(dz)) / footprint)
        columns["turns"] = turns
    elif profile == "transit":
        a = np.array(info["start"], dtype=float)
        b = np.array(info["end"], dtype=float)
        along_dir = (b - a) / max(np.linalg.norm(b - a), 1e-9)
        cross_dir = np.array([-along_dir[1], along_dir[0], 0.0])
        cross_dir /= max(np.linalg.norm(cross_dir), 1e-9)
        along = err @ along_dir
        cross = err @ cross_dir
        drift["final_along_m"] = float(along[-1])
        drift["final_cross_m"] = float(cross[-1])
        per_tower = []
        for k, x in enumerate(TOWERS_EAST, start=1):
            if min(a[0], b[0]) <= x <= max(a[0], b[0]):
                i = int(np.argmin(np.abs(G[:, 0] - x)))
                if abs(G[i, 0] - x) < 2.0:
                    per_tower.append({"tower": k, "dist_m": float(dist[i]), "error_m": float(norm[i]),
                                      "along_m": float(along[i]), "cross_m": float(cross[i]),
                                      "vertical_m": float(err[i, 2])})
        drift["per_tower"] = per_tower
        columns["along"] = along
        columns["cross"] = cross
    result["drift"] = drift

    lengths = [10, 25, 50, 100] if profile == "helix" else [50, 100, 200, 400]
    result["rpe"] = rpe(t, E, eyaw, G, gyaw, dist, [d for d in lengths if d < length])

    tum = {"t": t, "E": Ea, "G": G, "eyaw": eyaw + math.atan2(R[1, 0], R[0, 0]), "gyaw": gyaw}
    return result, columns, tum


# ─────────────────────────────────────────────────────────────────────────────
# output
# ─────────────────────────────────────────────────────────────────────────────

def report(name, r):
    s, d, h = r["segment"], r["drift"], r["health"]
    lines = [f"── {name}  [{r['profile']}]  {s['length_m']:.0f} m in {s['duration_s']:.0f} s, "
             f"{s['samples']} poses ──"]
    lines.append(f"  health     {h['rate_hz']:.1f} Hz, longest gap {h['longest_gap_s']:.2f} s, "
                 f"{h['jumps_over_1m']} jumps > 1 m"
                 + (f", STARTED {h['started_late_s']:.1f} s LATE" if h["started_late_s"] > 1.0 else ""))
    a = r["ate_posyaw"]
    lines.append(f"  ATE 4-DOF  rmse {a['rmse']:.3f} m, median {a['median']:.3f}, max {a['max']:.3f}, "
                 f"yaw rmse {a['yaw_rmse_deg']:.2f} deg, scale (est/true) {r['scale_sim3']:.4f}")
    lines.append(f"  drift      end {d['final_m']:.2f} m = {d['final_percent_of_distance']:.2f} % of "
                 f"distance (horiz {d['final_horizontal_m']:.2f}, vert {d['final_vertical_m']:+.2f}), "
                 f"max {d['max_m']:.2f} m, yaw {d['final_yaw_deg']:+.2f} deg")
    if r["profile"] == "helix":
        for p in d.get("per_turn", []):
            lines.append(f"    turn {p['turn']}  at {p['dist_m']:6.0f} m: {p['error_m']:.2f} m "
                         f"(vert {p['vertical_m']:+.2f}, radial {p['radial_m']:+.2f}, "
                         f"yaw {p['yaw_deg']:+.2f} deg)")
        if "overlap_worst_case" in d:
            lines.append(f"    view overlap: planned {100 * d['overlap_planned']:.0f} %, worst turn "
                         f"with the vertical drift {100 * d['overlap_worst_case']:.0f} % "
                         f"({d['worst_turn_to_turn_vertical_m']:.2f} m turn-to-turn)")
    else:
        lines.append(f"    at the end: along-track {d['final_along_m']:+.2f} m (scale), "
                     f"cross-track {d['final_cross_m']:+.2f} m")
        for p in d.get("per_tower", []):
            lines.append(f"    tower {p['tower']} at {p['dist_m']:6.0f} m: {p['error_m']:.2f} m "
                         f"(along {p['along_m']:+.2f}, cross {p['cross_m']:+.2f}, "
                         f"vert {p['vertical_m']:+.2f})")
    for k, v in r["rpe"].items():
        lines.append(f"  RPE {k:>5}  mean {v['mean']:.3f} m ({v['percent_of_length']:.2f} %), "
                     f"rmse {v['rmse']:.3f}, yaw {v['yaw_mean_deg']:.2f} deg  [{v['n']}]")
    return "\n".join(lines)


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def write_outputs(outdir, r, columns, tum):
    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump(r, f, indent=2)
    keys = list(columns)
    with open(os.path.join(outdir, "errors.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(keys)
        for row in zip(*(columns[k] for k in keys)):
            w.writerow([f"{v:.6f}" for v in row])
    # TUM: timestamp tx ty tz qx qy qz qw. The estimate is written already
    # 4-DOF aligned; orientation carries yaw only, which is all posyaw uses.
    for fname, P, Y in (("est_tum.txt", tum["E"], tum["eyaw"]), ("gt_tum.txt", tum["G"], tum["gyaw"])):
        with open(os.path.join(outdir, fname), "w") as f:
            f.write("# timestamp tx ty tz qx qy qz qw\n")
            for ti, p, y in zip(tum["t"], P, Y):
                q = yaw_to_quat(y)
                f.write(f"{ti:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                        f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")


def summarise(results):
    """Mean and spread across runs of the same profile."""
    def collect(fn):
        vals = []
        for r in results:
            try:
                vals.append(float(fn(r)))
            except (KeyError, TypeError, ValueError):
                pass
        return np.array(vals)
    rows = [("ATE rmse [m]", lambda r: r["ate_posyaw"]["rmse"]),
            ("end drift [m]", lambda r: r["drift"]["final_m"]),
            ("end drift [% dist]", lambda r: r["drift"]["final_percent_of_distance"]),
            ("end yaw drift [deg]", lambda r: abs(r["drift"]["final_yaw_deg"])),
            ("scale (est/true)", lambda r: r["scale_sim3"])]
    lines = [f"── across {len(results)} runs (mean ± std, min..max) ──"]
    for label, fn in rows:
        v = collect(fn)
        if v.size:
            lines.append(f"  {label:<22} {v.mean():8.3f} ± {v.std():.3f}   ({v.min():.3f} .. {v.max():.3f})")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bags", nargs="+", help="bag directories recorded by vio_trial.sh")
    ap.add_argument("--plot", action="store_true", help="also write eval/trajectory.png")
    ap.add_argument("--est-topic", default=EST_TOPIC)
    ap.add_argument("--gt-topic", default=GT_TOPIC)
    args = ap.parse_args()

    results, failed = [], 0
    for bag in args.bags:
        bag = bag.rstrip("/")
        name = os.path.basename(bag)
        try:
            est, gt, phases = read_bag(bag, args.est_topic, args.gt_topic)
            r, columns, tum = evaluate(est, gt, phases)
        except Exception as exc:  # one bad bag must not hide the others
            print(f"── {name}: NOT SCORED -- {exc}")
            failed += 1
            continue
        write_outputs(os.path.join(bag, "eval"), r, columns, tum)
        if args.plot:
            # Ground truth vs OpenVINS, whole flight: top view, side view, deviation.
            try:
                sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
                import plot_same_graph
                plot_same_graph.make(est, gt, phases, os.path.join(bag, "eval", "trajectory.png"))
            except Exception as exc:  # the plot is extra; never lose the score over it
                print(f"  (trajectory plot skipped: {exc})")
        print(report(name, r))
        print(f"  -> {os.path.join(bag, 'eval')}/")
        results.append(r)

    if len(results) > 1:
        for profile in sorted({r["profile"] for r in results}):
            same = [r for r in results if r["profile"] == profile]
            if len(same) > 1:
                print(f"[{profile}] " + summarise(same))
    return 1 if failed and not results else 0


if __name__ == "__main__":
    sys.exit(main())
