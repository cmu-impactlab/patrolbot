#!/usr/bin/env bash
# Read-only PatrolBot liveness/readiness report.
set -u

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
COMPOSE=(docker compose -f "$SCRIPT_DIR/docker-compose.yml" --env-file "$SCRIPT_DIR/.env")
CONTAINERS=(patrolbot-bringup patrolbot-bridge patrolbot-navigation)
SBC_HOST=${PATROLBOT_SBC_HOST:-10.0.0.1}
SBC_PORT=${PATROLBOT_SBC_PORT:-7272}

if [[ ! -f "$SCRIPT_DIR/.env" ]]; then
  COMPOSE=(docker compose -f "$SCRIPT_DIR/docker-compose.yml")
fi

printf 'PatrolBot container liveness\n'
all_healthy=true
for container in "${CONTAINERS[@]}"; do
  if ! state=$(docker inspect "$container" \
      --format 'status={{.State.Status}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} restarts={{.RestartCount}} image={{.Config.Image}}' \
      2>/dev/null); then
    printf '  %-24s missing\n' "$container"
    all_healthy=false
    continue
  fi
  printf '  %-24s %s\n' "$container" "$state"
  [[ "$state" == *"status=running health=healthy"* ]] || all_healthy=false
done

revision=$(docker inspect patrolbot-navigation \
  --format '{{index .Config.Labels "org.opencontainers.image.revision"}}' 2>/dev/null || true)
printf '  %-24s %s\n' "revision" "${revision:-unknown}"

bridge_publication_mode=$(docker inspect patrolbot-bridge \
  --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null \
  | sed -n 's/^RMW_FASTRTPS_PUBLICATION_MODE=//p' | head -n 1)
printf '  %-24s %s\n' "bridge DDS publication" \
  "${bridge_publication_mode:-unset}"

if [[ "$all_healthy" != true ]]; then
  printf '\nOVERALL=unhealthy reason=container-liveness\n'
  exit 1
fi

ros_exec() {
  local container=$1
  shift
  # The timeout MUST run inside the container. An outer `timeout docker exec`
  # kills only the exec client and can leave the ROS process running with an
  # unread output pipe. A leaked `ros2 topic hz` subscriber previously filled
  # that pipe and back-pressured the live /scan writer during navigation.
  docker exec "$container" timeout --signal=INT --kill-after=2 15 bash -lc \
    "source /opt/ros/\${ROS_DISTRO}/setup.bash; source /opt/patrolbot_ws/install/setup.bash; $*"
}

topic_is_fresh() {
  local topic=$1
  local output
  output=$(ros_exec patrolbot-bridge \
    "timeout --signal=INT --kill-after=2 8 ros2 topic hz --window 3 '$topic' 2>&1 || true") || return 1
  grep -q 'average rate:' <<<"$output"
}

topic_publisher_count() {
  local topic=$1 output count attempt
  for attempt in 1 2 3; do
    output=$(ros_exec patrolbot-navigation "ros2 topic info '$topic' 2>&1") || output=""
    count=$(sed -n 's/^Publisher count: //p' <<<"$output" | head -n 1)
    if [[ "$count" =~ ^[0-9]+$ ]]; then
      printf '%s\n' "$count"
      return 0
    fi
    sleep 1
  done
  return 1
}

topic_publisher_reliability() {
  local topic=$1 output reliability attempt
  for attempt in 1 2 3; do
    output=$(ros_exec patrolbot-navigation "ros2 topic info -v '$topic' 2>&1") || output=""
    reliability=$(awk '
      /Endpoint type: PUBLISHER/ { publisher = 1; next }
      publisher && /Reliability:/ { print $2; exit }
    ' <<<"$output")
    if [[ -n "$reliability" ]]; then
      printf '%s\n' "$reliability"
      return 0
    fi
    sleep 1
  done
  return 1
}

tf_is_ready() {
  local parent=$1 child=$2 output
  output=$(ros_exec patrolbot-navigation \
    "timeout --signal=INT --kill-after=2 8 ros2 run tf2_ros tf2_echo '$parent' '$child' 2>&1 || true") || return 1
  grep -q 'Translation:' <<<"$output"
}

lifecycle_is_active() {
  local container=$1 node=$2 output attempt
  # Retry with the ROS 2 daemon left running so discovery stays warm. The old
  # "daemon stop; sleep 1" gave fresh discovery only ~1s and produced false
  # "inactive" readings for nodes that were actually active.
  for attempt in 1 2 3 4 5 6; do
    output=$(ros_exec "$container" "ros2 lifecycle get '$node' 2>&1") || output=""
    if grep -q '^active ' <<<"$output"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

printf '\nSBC and ROS readiness\n'

# /odom freshness IS the SBC-connectivity signal: patrolbot_bridge publishes
# /odom only while connected to the SBC. We deliberately do NOT open a raw TCP
# socket to $SBC_HOST:$SBC_PORT -- the SBC server is single-client (it accepts
# the bridge and never accept()s again), so every probe connection piles up
# un-accepted in its listen backlog (CLOSE-WAIT) and, once the backlog fills,
# blocks the bridge's own reconnects. Inferring reachability from live /odom is
# both accurate and non-invasive.
ready=true

monitor_processes=""
for container in "${CONTAINERS[@]}"; do
  found=$(docker exec "$container" ps -eo pid,args 2>/dev/null \
    | awk '/[r]os2 topic hz/ { print }') || found=""
  if [[ -n "$found" ]]; then
    monitor_processes+="$container: $found"$'\n'
  fi
done
if [[ -n "$monitor_processes" ]]; then
  printf '  %-24s found (must be stopped)\n' "unattended topic monitors"
  printf '%s' "$monitor_processes"
  ready=false
else
  printf '  %-24s none\n' "unattended topic monitors"
fi

if [[ "$bridge_publication_mode" != ASYNCHRONOUS ]]; then
  printf '  %-24s %s (expected ASYNCHRONOUS)\n' \
    "bridge DDS publication" "${bridge_publication_mode:-unset}"
  ready=false
fi

if topic_is_fresh /odom; then
  printf '  %-24s fresh (SBC link up)\n' "/odom"
else
  printf '  %-24s unavailable/stale\n' "/odom"
  printf '  SBC link unavailable (bridge not receiving /odom)\n'
  printf '  telemetry and odom->base_link checks skipped\n'
  printf '\nOVERALL=degraded reason=sbc-unavailable\n'
  exit 2
fi

if topic_is_fresh /scan; then
  printf '  %-24s fresh\n' "/scan"
else
  printf '  %-24s unavailable/stale\n' "/scan"
  ready=false
fi

# One bridge publisher per hardware topic is a deployment invariant. This
# catches an accidentally re-enabled Pi 4 rollback stack before duplicate
# odom/scan streams can corrupt TF, costmaps, and navigation.
for topic in /odom /scan; do
  count=$(topic_publisher_count "$topic") || count=unknown
  if [[ "$count" == 1 ]]; then
    printf '  %-24s one\n' "$topic publisher"
  else
    printf '  %-24s %s (expected one)\n' "$topic publishers" "$count"
    ready=false
  fi
done

for topic in /scan /sonar; do
  reliability=$(topic_publisher_reliability "$topic") || reliability=unknown
  if [[ "$reliability" == BEST_EFFORT ]]; then
    printf '  %-24s BEST_EFFORT\n' "$topic publisher QoS"
  else
    printf '  %-24s %s (expected BEST_EFFORT)\n' \
      "$topic publisher QoS" "$reliability"
    ready=false
  fi
done

if lifecycle_is_active patrolbot-bringup /teleop_velocity_smoother; then
  printf '  %-24s active\n' "/teleop_velocity_smoother"
else
  printf '  %-24s inactive/unavailable\n' "/teleop_velocity_smoother"
  ready=false
fi

if lifecycle_is_active patrolbot-navigation /controller_server; then
  printf '  %-24s active\n' "/controller_server"
else
  printf '  %-24s inactive/unavailable\n' "/controller_server"
  ready=false
fi

for frames in "odom base_link" "base_link laser_frame"; do
  read -r parent child <<<"$frames"
  if tf_is_ready "$parent" "$child"; then
    printf '  %-24s ready\n' "$parent->$child"
  else
    printf '  %-24s unavailable\n' "$parent->$child"
    ready=false
  fi
done

if [[ "$ready" == true ]]; then
  printf '\nOVERALL=ready\n'
  exit 0
fi

printf '\nOVERALL=degraded reason=ros-readiness\n'
exit 2
