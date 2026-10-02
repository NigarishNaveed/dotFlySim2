# lib.sh -- shared by vio_trial.sh, record_sensor_check.sh and
# setup_openvins.sh. Sourced, never run. Talks to the dotflysim2 container the
# way walker does (walkerui/dockerctl.py): docker, or sudo docker when this
# user is not in the docker group.

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
CONTAINER="${WALKER_CONTAINER:-dotflysim2}"
HOST_BAGS="$REPO/bags"
# Paths INSIDE the container (repo dirs are mounted there by walker).
C_WS='$HOME/ws'
C_PROJECT='$HOME/ws/src/projects/vio_benchmark'
C_BAGS='$HOME/bags'

say()  { printf '\n==> %s\n' "$*"; }
warn() { printf '!!  %s\n' "$*" >&2; }
die()  { printf '%s: %s\n' "$(basename "$0")" "$*" >&2; exit 1; }

if docker info >/dev/null 2>&1; then
  ENGINE=(docker)
elif command -v sudo >/dev/null 2>&1; then
  ENGINE=(sudo docker)
else
  die "docker needs privileges this user does not have, and sudo is missing"
fi

# A ROS shell in the container. docker exec does not read .bashrc, so the
# overlay is sourced explicitly; set +u because ROS's setup files read unset vars.
_ros_prelude='set +u; source /opt/ros/jazzy/setup.bash; [ -f $HOME/ws/install/setup.bash ] && source $HOME/ws/install/setup.bash'

c()        { "${ENGINE[@]}" exec -i -u developer "$CONTAINER" bash -c "$_ros_prelude; $1"; }
c_bare()   { "${ENGINE[@]}" exec -i -u developer "$CONTAINER" bash -c "$1"; }
c_detach() { "${ENGINE[@]}" exec -d -u developer "$CONTAINER" bash -c "$_ros_prelude; $1"; }
wctl()     { c_bare "python3 $C_PROJECT/scripts/walkerctl.py $*"; }

container_up() {
  [ "$("${ENGINE[@]}" inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = "true" ]
}

require_walkerd() {
  container_up || die "the $CONTAINER container is not running -- start it with: cd $REPO && ./walker up"
  wctl ping >/dev/null || die "walkerd is not answering in $CONTAINER -- try: cd $REPO && ./walker up --restart-walkerd"
}

wrapper_alive() { c "timeout 8 ros2 topic echo --once /wrapper/psdk_ros2/flight_status >/dev/null 2>&1"; }

# Simulation (powerline, m4e, forward fisheye pair) and bridge, through
# walkerd. The camera set is locked while the sim runs, so a sim already up
# without the fisheye pair is restarted.
ensure_stack() {
  local sim cams
  sim="$(wctl state sim)"
  cams="$(c_bare "python3 $C_PROJECT/scripts/walkerctl.py state" | python3 -c 'import json,sys; print(json.load(sys.stdin)["selection"].get("cameras",""))' 2>/dev/null || true)"
  if [ "$sim" = "running" ] && [ "$cams" != "fisheye" ]; then
    warn "the simulation is running with cameras=$cams; OpenVINS needs the fisheye pair -- restarting it"
    wctl stop bridge >/dev/null || true
    wctl stop sim >/dev/null || true
    wctl wait sim stopped,failed 120 >/dev/null || true
    sim=stopped
  fi
  if [ "$sim" != "running" ]; then
    say "starting the simulation: powerline, m4e, fisheye cameras"
    wctl select world=powerline drone=m4e cameras=fisheye >/dev/null || die "walkerd refused the selection"
    wctl start sim >/dev/null || die "walkerd refused to start the simulation"
    wctl wait sim running 300 >/dev/null || die "the simulation did not come up -- see walker's sim terminal"
  fi
  if [ "$(wctl state bridge)" != "running" ]; then
    say "starting the bridge"
    wctl start bridge >/dev/null || die "walkerd refused to start the bridge"
    wctl wait bridge running 120 >/dev/null || die "the bridge did not come up"
  fi
  say "waiting for wrapper telemetry"
  local i
  for i in $(seq 1 30); do wrapper_alive && return 0; sleep 2; done
  die "the bridge is running but /wrapper/psdk_ros2/flight_status is silent"
}

restart_stack() {
  wctl stop bridge >/dev/null || true
  wctl stop sim >/dev/null || true
  wctl wait sim stopped,failed 120 >/dev/null || true
  ensure_stack
}

start_perception() {
  c "timeout 20 ros2 service call /wrapper/psdk_ros2/start_perception \
       psdk_interfaces/srv/PerceptionStereoVisionSetup \
       '{stereo_cameras_direction: FRONT, start_stop: true}' >/dev/null"
}

stereo_ok() { c "python3 $C_PROJECT/scripts/stereo_live_check.py 20"; }
