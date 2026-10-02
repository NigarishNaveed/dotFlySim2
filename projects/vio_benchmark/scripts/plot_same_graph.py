#!/usr/bin/env python3
"""
plot_same_graph.py -- ONE figure per flight: ground truth and OpenVINS on the
same axes, whole flight (takeoff to landing), and the deviation between them.

    top view      east vs north, both paths
    side view     east vs height, both paths
    deviation     |OpenVINS - ground truth| in metres, over the flight

OpenVINS keeps its own map (origin where it started, its own zero heading), so
it is lined up with the ground truth ONCE, at its first pose -- position and
heading -- and then left alone: what is plotted is what it would have told the
aircraft from takeoff on.

vio_eval.py --plot calls make() for every scored bag. By hand:

    python3 plot_same_graph.py <dir with est.npy gt.npy ph.json> <out.png>

est/gt: arrays of rows [t, x, y, z, qx, qy, qz, qw]; ph: the /vio_benchmark/phase
JSON lines (used only to know when the flight ended).
"""
import json
import math
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TRUTH, OPENVINS = "black", "#d9480f"


def _yaw(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def make(est, gt, ph, out_png):
    """Write the figure. Returns (flight seconds, final deviation m, max deviation m)."""
    # The flight only: once diverged, OpenVINS's own camera-IMU time-offset
    # estimate runs away and it stamps poses far past the end of the flight.
    t_end = max(p["t"] for p in ph) if ph else gt[-1, 0]
    est = est[(est[:, 0] >= gt[0, 0]) & (est[:, 0] <= min(t_end, gt[-1, 0]))]
    if len(est) < 2:
        raise ValueError("no OpenVINS poses inside the flight")

    # ground truth at every OpenVINS stamp, and OpenVINS lined up at its first pose
    G = np.column_stack([np.interp(est[:, 0], gt[:, 0], gt[:, k]) for k in (1, 2, 3)])
    gyaw = np.interp(est[:, 0], gt[:, 0], np.unwrap([_yaw(q) for q in gt[:, 4:8]]))
    d = gyaw[0] - _yaw(est[0, 4:8])
    R = np.array([[math.cos(d), -math.sin(d), 0], [math.sin(d), math.cos(d), 0], [0, 0, 1]])
    E = (est[:, 1:4] - est[0, 1:4]) @ R.T + G[0]
    dev = np.linalg.norm(E - G, axis=1)
    t = est[:, 0] - est[0, 0]

    # the whole ground-truth path of the flight, at full rate
    gsel = (gt[:, 0] >= est[0, 0]) & (gt[:, 0] <= est[-1, 0])
    T = gt[gsel, 1:4]

    fig = plt.figure(figsize=(15, 11))
    fig.suptitle("Ground truth (black) vs OpenVINS (orange) — whole flight, same axes, metres\n"
                 "OpenVINS lined up with the ground truth once, at its first pose", fontsize=12)
    grid = fig.add_gridspec(2, 2, height_ratios=[1.15, 1])

    for col, (yi, title, ylabel) in enumerate(((1, "Top view", "north [m]"), (2, "Side view", "height [m]"))):
        a = fig.add_subplot(grid[0, col])
        a.plot(T[:, 0], T[:, yi], color=TRUTH, lw=2.2, label="ground truth")
        a.plot(E[:, 0], E[:, yi], color=OPENVINS, lw=1.6, label="OpenVINS")
        a.plot(T[0, 0], T[0, yi], "o", color="green", ms=7, label="takeoff")
        a.set_title(title); a.set_xlabel("east [m]"); a.set_ylabel(ylabel)
        a.grid(alpha=0.3); a.legend(loc="best")

    a = fig.add_subplot(grid[1, :])
    a.plot(t, dev, color=OPENVINS, lw=1.6)
    a.fill_between(t, 0, dev, color=OPENVINS, alpha=0.12)
    a.set_title(f"Deviation between the two paths — final {dev[-1]:.1f} m, max {dev.max():.1f} m")
    a.set_xlabel("time since takeoff [s]"); a.set_ylabel("deviation [m]")
    a.set_xlim(0, t[-1]); a.set_ylim(bottom=0); a.grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_png, dpi=100)
    plt.close(fig)
    return float(t[-1]), float(dev[-1]), float(dev.max())


if __name__ == "__main__":
    D, OUT = sys.argv[1], sys.argv[2]
    secs, final, worst = make(np.load(D + "/est.npy"), np.load(D + "/gt.npy"),
                              json.load(open(D + "/ph.json")), OUT)
    print(f"{OUT}: {secs:.0f} s flight, deviation final {final:.1f} m, max {worst:.1f} m")
