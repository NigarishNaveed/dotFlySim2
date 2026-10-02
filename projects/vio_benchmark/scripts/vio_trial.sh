#!/usr/bin/env bash
# =============================================================================
# vio_trial.sh -- one OpenVINS trial, start to score. Run from the HOST with
# the container up (./walker up; the walker dashboard may stay open):
#
#     ./projects/vio_benchmark/scripts/vio_trial.sh helix
#     ./projects/vio_benchmark/scripts/vio_trial.sh transit --runs 3
#     ./projects/vio_benchmark/scripts/vio_trial.sh helix --name ov20 -- helix.overlap=0.2
#
# Drives the same walkerd units walker's dashboard does:
#   1. sim (powerline, m4e, cameras=fisheye) and bridge, started if not up
#   2. per run: stereo pair on and CHECKED (left != right; a broken pair
#      restarts the sim once), OpenVINS started FRESH, recorder started,
#      the vio_benchmark mission started with profile=<helix|transit>,
#      waited for, then recorder and OpenVINS stopped
#   3. scoring: vio_eval.py -> bags/<run>/eval/ (summary, plots, trajectory
#      plots of real vs OpenVINS on the same axes)
#
# Options:
#   --runs N     fly the profile N times (default 1)
#   --name TAG   put TAG in the bag names
#   --rviz       open OpenVINS's own RViz view
#   --images     also record the stereo pair and the IMU (~20 MB/s), so
#                OpenVINS can be re-run on the same flight with other settings
#   --no-eval    record only
#   -- k=v ...   mission parameters, e.g. -- helix.radius=15 transit.speed=8
#
# Needs, once: scripts/setup_openvins.sh.
# =============================================================================
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/lib.sh"
CONFIG="$C_PROJECT/config/estimator_config.yaml"

usage() { sed -n '3,30p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit "${1:-0}"; }

[ "$#" -ge 1 ] || usage 1
profile="$1"; shift
case "$profile" in
  helix|transit) ;;
  -h|--help) usage 0 ;;
  *) die "profile must be helix or transit, got '$profile'" ;;
esac

runs=1 tag="" rviz=false images=0 do_eval=1 extra=()
while [ "$#" -gt 0 ]; do
  case "$1" in
    --runs)    shift; runs="${1:-}"; [[ "$runs" =~ ^[1-9][0-9]*$ ]] || die "--runs needs a number" ;;
    --name)    shift; tag="${1:-}"; [[ "$tag" =~ ^[A-Za-z0-9_.-]+$ ]] || die "--name: letters, digits, _ . - only" ;;
    --rviz)    rviz=true ;;
    --images)  images=1 ;;
    --no-eval) do_eval=0 ;;
    --)        shift; extra=("$@"); break ;;
    -h|--help) usage 0 ;;
    *)         die "unknown option '$1' (mission parameters go after --, as key=value)" ;;
  esac
  shift
done
for kv in "${extra[@]}"; do [[ "$kv" == *=* ]] || die "mission parameter '$kv' is not key=value"; done

stop_recorder() {
  c_bare "pkill -INT -f '[r]os2 bag record' || true
          for i in \$(seq 40); do pgrep -f '[r]os2 bag record' >/dev/null || exit 0; sleep 0.5; done
          pkill -KILL -f '[r]os2 bag record' || true" || true
}
stop_openvins() {
  c_bare "pkill -INT -f '[s]ubscribe.launch.py' || true; pkill -INT -f '[r]un_subscribe_msckf' || true
          for i in \$(seq 20); do pgrep -f '[r]un_subscribe_msckf' >/dev/null || exit 0; sleep 0.5; done
          pkill -KILL -f '[r]un_subscribe_msckf' || true" || true
}
on_interrupt() {
  warn "interrupted -- stopping the mission, the recorder and OpenVINS"
  wctl stop project >/dev/null 2>&1 || true
  stop_recorder; stop_openvins
  exit 130
}
trap on_interrupt INT TERM

# ---- preflight -------------------------------------------------------------------
require_walkerd
c_bare "test -x $C_WS/install/ov_msckf/lib/ov_msckf/run_subscribe_msckf" \
  || die "OpenVINS is not built in $CONTAINER -- run: $REPO/projects/vio_benchmark/scripts/setup_openvins.sh"
[ "$(wctl state project)" = "running" ] && die "a mission is already running -- stop it in walker (or: wait for it) first"

ensure_stack

say "checking the ground truth is published"
c "timeout 10 ros2 topic echo --once /drone/ground_truth/odom >/dev/null" \
  || die "nothing on /drone/ground_truth/odom. The OdometryPublisher in models/m4e/model.sdf and its
    row in tools/compose_sim.py are read when the simulation STARTS -- stop the sim in walker and
    start it again."

stop_openvins; stop_recorder      # leftovers from an interrupted trial

topics="/ov_msckf/poseimu /ov_msckf/odomimu /drone/ground_truth/odom /vio_benchmark/phase /wrapper/psdk_ros2/position_fused"
if [ "$images" = "1" ]; then
  topics+=" /wrapper/psdk_ros2/perception_stereo_left_stream /wrapper/psdk_ros2/perception_stereo_right_stream /wrapper/psdk_ros2/imu"
fi

# ---- the runs --------------------------------------------------------------------
bags=()
for run in $(seq 1 "$runs"); do
  bag="vio_${profile}${tag:+_$tag}_$(date +%Y-%m-%d_%H-%M-%S)"
  say "run $run/$runs: $bag"

  # The pair must stream BEFORE takeoff: OpenVINS initialises on the takeoff
  # jolt from stationary frames just before it.
  start_perception || warn "start_perception call failed -- the mission will try again"

  # Never fly on a broken pair: one dotFlySim session delivered the SAME image
  # on both streams, and the estimator diverged in seconds. Restart once.
  if ! stereo_ok; then
    warn "the stereo pair is broken (identical or missing frames) -- restarting the simulation once"
    restart_stack
    start_perception || true
    stereo_ok || die "the stereo pair is still broken after a restart -- not flying. Run
    ./projects/vio_benchmark/scripts/record_sensor_check.sh and send the result."
  fi

  c_detach "ros2 launch ov_msckf subscribe.launch.py config_path:=$CONFIG \
      verbosity:=WARNING rviz_enable:=$rviz > /tmp/vio_trial_openvins.log 2>&1"
  c "for i in \$(seq 30); do ros2 node list 2>/dev/null | grep -q ov_msckf && exit 0; sleep 1; done; exit 1" || {
    warn "OpenVINS did not come up. Last lines of /tmp/vio_trial_openvins.log:"
    c_bare "grep -v class_loader /tmp/vio_trial_openvins.log | tail -25" || true
    stop_openvins
    die "OpenVINS did not come up -- the lines above say why"
  }

  c_detach "ros2 bag record -s mcap --disable-keyboard-controls -o $C_BAGS/$bag --topics $topics \
      > /tmp/vio_trial_record.log 2>&1"
  sleep 3

  say "flying the $profile (walker's project unit: vio_benchmark)"
  rebuild=--rebuild; [ "$run" -gt 1 ] && rebuild=--no-rebuild
  wctl project vio_benchmark "profile=$profile" "${extra[@]}" "$rebuild" >/dev/null \
    || die "walkerd refused to start the mission"
  wctl wait project starting,running 600 >/dev/null || warn "the mission never started -- see walker's project terminal"
  # The whole sortie at a real-time factor that can drop to ~0.5 with the
  # cameras on: allow an hour of wall time before giving up.
  wctl wait project stopped,failed 3600 >/dev/null || warn "the mission did not finish in time"
  [ "$(wctl state project)" = "failed" ] && warn "the mission FAILED -- see walker's project terminal (the bag is kept)"

  say "PX4's latest arming / preflight messages"
  c_bare "timeout 15 tail -c 400000 /tmp/walker/sim.log 2>/dev/null | grep -aE 'Preflight|[Aa]rming|[Dd]enied|Ready for takeoff|[Tt]akeoff' | tail -12" \
    || true

  stop_recorder
  stop_openvins
  bags+=("$C_BAGS/$bag")
  say "recorded $HOST_BAGS/$bag"
  if [ "$run" -lt "$runs" ]; then sleep 5; fi
done

# ---- score -----------------------------------------------------------------------
if [ "$do_eval" = "1" ]; then
  say "scoring"
  c "python3 $C_PROJECT/scripts/vio_eval.py --plot ${bags[*]}" || warn "the evaluator reported a problem (above)"
  say "results: $HOST_BAGS/<run>/eval/  (summary.json, trajectory.png, errors.csv)"
fi
