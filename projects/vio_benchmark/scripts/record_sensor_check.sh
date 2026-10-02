#!/usr/bin/env bash
# =============================================================================
# record_sensor_check.sh -- 20 s of exactly what OpenVINS is fed, aircraft on
# the ground, to check the INPUTS rather than the estimator. From the HOST,
# with the container up (./walker up):
#
#     ./projects/vio_benchmark/scripts/record_sensor_check.sh [SECONDS]
#
# Starts the sim (powerline, m4e, fisheye) and bridge through walkerd if they
# are not up, switches the stereo pair on, and records /wrapper/psdk_ros2/imu,
# both perception streams, the same two cameras straight out of Gazebo
# (/drone/perception/front/*/image_raw, before the bridge) and the ground truth
# to bags/vio_sensorcheck_<time>/ (~700 MB). Then runs check_sensors.py on it:
# gravity on z (FLU), IMU spacing and clock, left != right, which way up,
# trackable corners (including on the ground).
# =============================================================================
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
SECONDS_TO_RECORD="${1:-20}"
bag="vio_sensorcheck_$(date +%Y-%m-%d_%H-%M-%S)"

require_walkerd
ensure_stack

say "switching the stereo pair on"
start_perception || warn "start_perception failed"
sleep 3

say "recording ${SECONDS_TO_RECORD} s to bags/$bag"
# DETACHED and without keyboard controls: run in the foreground through a
# terminal, ros2 bag record (dotFlySim) started its SPACE-to-pause handler,
# never subscribed, and ignored SIGINT until killed.
c_detach "ros2 bag record -s mcap --disable-keyboard-controls -o $C_BAGS/$bag \
  --topics /wrapper/psdk_ros2/imu \
  /wrapper/psdk_ros2/perception_stereo_left_stream \
  /wrapper/psdk_ros2/perception_stereo_right_stream \
  /drone/perception/front/left/image_raw \
  /drone/perception/front/right/image_raw \
  /drone/ground_truth/odom > /tmp/sensor_check_record.log 2>&1"
sleep "$SECONDS_TO_RECORD"
c_bare "pkill -INT -f '[r]os2 bag record' || true
  for i in \$(seq 30); do pgrep -f '[r]os2 bag record' >/dev/null || exit 0; sleep 0.5; done
  pkill -KILL -f '[r]os2 bag record' || true" || true

[ -d "$HOST_BAGS/$bag" ] || {
  warn "no recording was written. The recorder said:"
  c_bare "tail -30 /tmp/sensor_check_record.log" >&2 || true
  exit 1
}
say "recorded bags/$bag -- checking it"
c "python3 $C_PROJECT/scripts/check_sensors.py $C_BAGS/$bag" || true
