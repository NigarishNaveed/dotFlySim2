# vio_benchmark — scoring OpenVINS on a helix and a transit

The two trajectories a powerline inspection is made of, flown the same way
every time. OpenVINS runs on the forward "fisheye" stereo pair and the 50 Hz
wrapper IMU, and its estimate is scored against Gazebo's exact pose.

| Profile | What is flown | What it stresses |
|---|---|---|
| `helix` (default) | Round `tower_1` (x=60) at 12 m radius, 5 → 36 m, nose on the tower, each turn's camera view overlapping the one below by 30 % | Rotation-dominant motion, a thin lattice against the sky, turn-to-turn vertical drift |
| `transit` | Straight down the centreline at 42 m, x=40 → 880 (840 m, four spans) | Forward motion, far features, the 9 cm baseline, scale |

The mission itself is wrapper-surface only, like every project here. It flies
closed loop on `position_fused`, checks the yaw-rate sign in the air, and won't
fly the measured segment until OpenVINS is publishing. Ground truth is read only
by the scoring, never by the mission.

## Once

```bash
docker build -t dotflysim2:latest .               # if not built yet (~40 min)
./walker up                                       # container + supervisor
./projects/vio_benchmark/scripts/setup_openvins.sh   # clone, patch, build OpenVINS (minutes)
```

The OpenVINS **source** lives in `projects/open_vins/` on the host and survives
the container. The **build** doesn't: after `./walker down --rm`, run the setup
script again. It skips the clone and only rebuilds.

## Check the inputs (1 minute, no flight)

```bash
./projects/vio_benchmark/scripts/record_sensor_check.sh
```

This records 20 s of exactly what OpenVINS is fed, then prints PASS/FAIL for:
- gravity on z (FLU axes) and IMU timing;
- IMU and images on one clock;
- left image ≠ right image;
- the image the right way up;
- trackable corners, **including on the ground**.

## Each trial

```bash
./projects/vio_benchmark/scripts/vio_trial.sh helix
./projects/vio_benchmark/scripts/vio_trial.sh transit --runs 3
./projects/vio_benchmark/scripts/vio_trial.sh helix --name r15 -- helix.radius=15
```

The script drives walkerd, so the units show up on the walker dashboard as usual:
1. **Stack:** starts the sim (powerline, m4e, `cameras=fisheye`) and the bridge
   if they aren't up. A sim running without the fisheye pair is restarted.
2. **Each run:**
   - turns the stereo pair on and checks it. If the frames are identical, it
     restarts the sim once.
   - starts OpenVINS fresh and starts recording to `bags/vio_<profile>_…`.
   - runs the `vio_benchmark` mission with `profile=<…>` and waits for it to
     land.
   - prints PX4's arming messages.
3. **Scoring:** results go to `bags/<run>/eval/`:

| File | Contents |
|---|---|
| `summary.json`, `errors.csv` | ATE, drift per turn or per tower, RPE, scale, estimator health |
| `trajectory.png` | the whole flight, ground truth and OpenVINS on the same axes: top view, side view, and the deviation between the two over time |
| `est_tum.txt`, `gt_tum.txt` | trajectories for `ov_eval` / `evo` |

Picked from walker's mission list (key `p`), the mission flies the default
profile, `helix`: walker passes no ROS parameters.

## Files

- `src/main.cpp`: the mission.
- `project.conf`: `CAMERAS=fisheye`, `SETPOINT=velocity`.
- `config/`: OpenVINS settings. The camera calibration is pinhole with zero
  distortion, because the "fisheye" sensors render rectilinear 90° images
  (`models/m4e/model.sdf`, `config/perception_calib.yaml`). The IMU noise comes
  from `IMU_CALIBRATION.md`.
- `scripts/`:
  - `vio_trial.sh`: runs a whole trial.
  - `record_sensor_check.sh` + `check_sensors.py`: check the inputs.
  - `setup_openvins.sh`: clone, patch and build OpenVINS.
  - `vio_eval.py` + `plot_same_graph.py`: scoring and the trajectory plot.
  - `stereo_live_check.py`: checks the stereo pair before each flight.
  - `walkerctl.py`, `lib.sh`: helpers for talking to walkerd and the container.

## Changes outside this folder, and why

- **`models/m4e/model.sdf`, `tools/compose_sim.py`:** add the ground-truth
  OdometryPublisher, bridged as `/drone/ground_truth/odom`. All three of its
  topics are renamed, because its default covariance topic is the one PX4 fuses
  as external vision.
- **`bridge/simty/converters.py`:** the IMU is now stamped from PX4's sample
  time (`_px4_sample_stamp`). Before, it was stamped with the latest `/clock`
  at conversion, which bunched samples onto one stamp whenever the bridge fell
  behind. Measured in dotFlySim: 81 of 499 samples repeated their predecessor's
  stamp.
- **`models/m4e/walker.toml`:** PX4's preflight GPS drift limits are raised to
  their maximums. The white GPS noise otherwise blocks takeoff.
- **`worlds/powerline.sdf`, `models/ground_scrub/`, `tools/gen_ground_texture.py`:**
  a textured ground. On the old flat colour, OpenVINS found 0 trackable corners
  below the horizon.
- **`config/perception_calib.yaml`:** the stereo and IMU numbers in one place.
  It was referenced, but missing.

## Things that will bite you

- **The IMU is 50 Hz,** and the spacing is uneven. Hard manoeuvres hurt more
  than they would with a 200 Hz IMU.
- **The cameras render only to 200 m** (`<far>`), and the baseline is 9 cm.
  Depth from stereo is poor beyond about 10 m, so it comes mostly from motion.
- **Gazebo slows down with the cameras on** (real-time factor about 0.5). The
  mission counts ticks; the trial allows an hour of wall time.
- **PX4 is unpinned in the Dockerfile.** Rebuilding the image can pull new
  parameter names; `walker.toml` sets both spellings of the drift limits.
