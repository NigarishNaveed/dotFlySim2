#!/usr/bin/env bash
# =============================================================================
# setup_openvins.sh -- fetch, patch and build OpenVINS for dotFlySim2. Run once,
# from the HOST, with the container up (./walker up):
#
#     ./projects/vio_benchmark/scripts/setup_openvins.sh
#
# WHERE IT GOES. projects/open_vins/ on the host = ~/ws/src/projects/open_vins
# in the container: projects/ is mounted, so the SOURCE survives the container.
# The BUILD (~/ws/build, ~/ws/install) does not: after `./walker down --rm`
# run this again (it skips the clone and only rebuilds). It has no package.xml
# at its top, so walker does not list it as a mission (walkerui/scan.py).
#
# TWO PATCHES, both found the hard way in dotFlySim:
#   1. QoS. OpenVINS subscribes to its cameras RELIABLE; the bridge publishes
#      the perception streams BEST_EFFORT (QoS "video"), and DDS never matches
#      the two -- no error, just zero images and an estimator that never starts.
#   2. Jazzy header names. OpenVINS targets Humble; Jazzy ships only the .hpp
#      image_transport header ("image_transport/image_transport.h: No such
#      file") and deprecates the others. Every line touched is in its ROS 2 branch.
#
# Not while the simulation is flying, if you can help it: the build saturates
# the CPU. JOBS=1 if it runs out of memory.
# =============================================================================
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"

OV="$REPO/projects/open_vins"
JOBS="${JOBS:-2}"
OV_URL="${OV_URL:-https://github.com/rpng/open_vins.git}"
OV_REF="${OV_REF:-master}"

container_up || die "the $CONTAINER container is not running -- start it with: cd $REPO && ./walker up"

# ---- 1. source ---------------------------------------------------------------
if [ -d "$OV/.git" ]; then
  say "OpenVINS already in $OV ($(git -C "$OV" rev-parse --short HEAD)) -- not re-cloning"
else
  say "cloning $OV_URL ($OV_REF) into $OV"
  git clone --branch "$OV_REF" "$OV_URL" "$OV"
fi

# ---- 2a. QoS patch (idempotent) ----------------------------------------------
VIZ="$OV/ov_msckf/src/ros/ROS2Visualizer.cpp"
[ -f "$VIZ" ] || die "$VIZ not found -- has OpenVINS moved its ROS 2 code?"
if grep -q 'dotFlySim' "$VIZ"; then
  say "QoS patch already applied"
else
  say "patching the image subscribers to best-effort (SensorDataQoS)"
  sed -i \
    -e 's|(_node, cam_topic0);|(_node, cam_topic0, rmw_qos_profile_sensor_data); // dotFlySim: best-effort streams|' \
    -e 's|(_node, cam_topic1);|(_node, cam_topic1, rmw_qos_profile_sensor_data); // dotFlySim: best-effort streams|' \
    -e 's|cam_topic, 10, \[this, i\]|cam_topic, rclcpp::SensorDataQoS(), [this, i] /* dotFlySim */|' \
    "$VIZ"
  n="$(grep -c 'dotFlySim' "$VIZ" || true)"
  [ "$n" -eq 3 ] || die "expected to patch 3 lines in $VIZ, patched $n -- upstream changed; patch by hand (see the top of this script)"
fi

# ---- 2b. Jazzy header names (idempotent) -------------------------------------
say "updating ROS 2 header names for Jazzy (.h -> .hpp)"
sed -i \
  -e 's|<image_transport/image_transport\.h>|<image_transport/image_transport.hpp>|' \
  -e 's|<cv_bridge/cv_bridge\.h>|<cv_bridge/cv_bridge.hpp>|' \
  -e 's|<tf2_geometry_msgs/tf2_geometry_msgs\.h>|<tf2_geometry_msgs/tf2_geometry_msgs.hpp>|' \
  -e 's|<tf2_ros/transform_broadcaster\.h>|<tf2_ros/transform_broadcaster.hpp>|' \
  "$OV/ov_msckf/src/ros/ROS2Visualizer.h" \
  "$OV/ov_msckf/src/ros/ROSVisualizerHelper.h"

# ---- 3. build ----------------------------------------------------------------
say "building ov_msckf, ov_eval and vio_benchmark in $CONTAINER (JOBS=$JOBS) -- several minutes"
c "cd $C_WS && MAKEFLAGS=-j$JOBS colcon build \
    --packages-up-to ov_msckf ov_eval vio_benchmark \
    --parallel-workers 1 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release"

c_bare "test -x $C_WS/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf" \
  || die "build finished but run_subscribe_msckf is missing -- read the colcon output above"

say "done. Next: ./projects/vio_benchmark/scripts/record_sensor_check.sh   (checks the inputs)"
say "then:       ./projects/vio_benchmark/scripts/vio_trial.sh helix"
