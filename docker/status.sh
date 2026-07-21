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

image_source_digest=$(docker inspect patrolbot-navigation \
  --format '{{index .Config.Labels "io.patrolbot.source-tree-sha256"}}' \
  2>/dev/null || true)
deployed_source_digest=$(
  "$SCRIPT_DIR/source-tree-digest.sh" "${PATROLBOT_ROOT:-$SCRIPT_DIR/..}" \
  2>/dev/null || true)
printf '  %-24s %s\n' "image source digest" \
  "${image_source_digest:-unset}"
printf '  %-24s %s\n' "deployed source digest" \
  "${deployed_source_digest:-unavailable}"
source_digest_ok=false
if [[ -n "$image_source_digest" && \
      "$image_source_digest" != unknown && \
      "$image_source_digest" == "$deployed_source_digest" ]]; then
  source_digest_ok=true
fi

discovery_ok=true
for container in "${CONTAINERS[@]}"; do
  discovery_range=$(docker inspect "$container" \
    --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null \
    | sed -n 's/^ROS_AUTOMATIC_DISCOVERY_RANGE=//p' | head -n 1)
  printf '  %-24s %s\n' "$container discovery" \
    "${discovery_range:-unset}"
  [[ "$discovery_range" == LOCALHOST ]] || discovery_ok=false
done

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

topic_endpoint_nodes() {
  local topic=$1 endpoint_type=$2 output nodes attempt
  for attempt in 1 2 3; do
    output=$(ros_exec patrolbot-navigation "ros2 topic info -v '$topic' 2>&1") || output=""
    nodes=$(awk -v endpoint_type="$endpoint_type" '
      /^Node name:/ { node = $3 }
      /^Endpoint type:/ && $3 == endpoint_type { print node }
    ' <<<"$output" | sort -u | paste -sd, -)
    if [[ -n "$nodes" ]]; then
      printf '%s\n' "$nodes"
      return 0
    fi
    if [[ -n "$output" && "$output" != *"Unknown topic"* && "$attempt" -eq 3 ]]; then
      printf '\n'
      return 0
    fi
    sleep 1
  done
  return 1
}

ros_name_exists() {
  local kind=$1 name=$2 output
  output=$(ros_exec patrolbot-navigation "ros2 '$kind' list 2>&1") || return 1
  grep -Fxq "$name" <<<"$output"
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

if [[ "$source_digest_ok" != true ]]; then
  printf '  %-24s mismatch (image must match deployed source)\n' \
    "source provenance"
  ready=false
else
  printf '  %-24s verified\n' "source provenance"
fi

if [[ "$discovery_ok" != true ]]; then
  printf '  %-24s invalid (expected LOCALHOST in every container)\n' \
    "DDS discovery scope"
  ready=false
else
  printf '  %-24s LOCALHOST only\n' "DDS discovery scope"
fi

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

printf '\nPi safety architecture\n'

for specification in \
    "/input/safety_controller PUBLISHER patrolbot_safety_watchdog" \
    "/input/nav_controller PUBLISHER controller_server" \
    "/input/nav_controller SUBSCRIPTION nav_command_mux" \
    "/input/nav_behavior PUBLISHER behavior_server" \
    "/input/nav_behavior SUBSCRIPTION nav_command_mux" \
    "/input/docking PUBLISHER " \
    "/input/docking SUBSCRIPTION nav_command_mux" \
    "/cmd_vel_nav PUBLISHER nav_command_mux" \
    "/cmd_vel_nav SUBSCRIPTION velocity_smoother" \
    "/cmd_vel_out PUBLISHER cmd_vel_mux" \
    "/cmd_vel_out SUBSCRIPTION patrolbot_safety_watchdog,teleop_velocity_smoother" \
    "/cmd_vel PUBLISHER teleop_velocity_smoother" \
    "/cmd_vel SUBSCRIPTION patrolbot_bridge"; do
  read -r topic endpoint expected <<<"$specification"
  actual=$(topic_endpoint_nodes "$topic" "$endpoint") || actual=unknown
  if [[ "$actual" == "${expected:-}" ]]; then
    printf '  %-24s %s\n' "$topic $endpoint" "${actual:-none}"
  else
    printf '  %-24s %s (expected %s)\n' \
      "$topic $endpoint" "${actual:-none}" "${expected:-none}"
    ready=false
  fi
done

if ros_name_exists node /docking_server; then
  printf '  %-24s PRESENT (must remain disabled)\n' "/docking_server"
  ready=false
else
  printf '  %-24s absent\n' "/docking_server"
fi

if ros_name_exists topic /scan_unfiltered_candidate; then
  printf '  %-24s PRESENT (must remain uncommissioned)\n' \
    "/scan_unfiltered_candidate"
  ready=false
else
  printf '  %-24s absent/uncommissioned\n' \
    "/scan_unfiltered_candidate"
fi

smoother_max=$(ros_exec patrolbot-bringup \
  "ros2 param get /teleop_velocity_smoother max_velocity 2>&1" || true)
if [[ "$smoother_max" == *"[0.35, 0.0, 0.7]"* ]]; then
  printf '  %-24s [0.35, 0.0, 0.7]\n' "teleop smoother max"
else
  printf '  %-24s %s (unexpected)\n' "teleop smoother max" \
    "${smoother_max:-unavailable}"
  ready=false
fi

battery_stop=$(ros_exec patrolbot-navigation \
  "ros2 param get /patrolbot_safety_watchdog battery_stop_voltage 2>&1" || true)
bumper_enforcement=$(ros_exec patrolbot-navigation \
  "ros2 param get /patrolbot_safety_watchdog bumper_enforcement_enabled 2>&1" || true)
if [[ "$battery_stop" == *"23.0"* ]]; then
  printf '  %-24s 23.0 V\n' "battery motion stop"
else
  printf '  %-24s %s (expected 23.0 V)\n' "battery motion stop" \
    "${battery_stop:-unavailable}"
  ready=false
fi
if [[ "$bumper_enforcement" == *"False"* ]]; then
  printf '  %-24s disabled/uncommissioned\n' "bumper enforcement"
else
  printf '  %-24s %s (requires physical mapping review)\n' \
    "bumper enforcement" "${bumper_enforcement:-unavailable}"
  ready=false
fi

if docker exec patrolbot-navigation test -e /dev/input/js0 2>/dev/null; then
  if topic_is_fresh /joy; then
    printf '  %-24s receiver present; /joy fresh\n' "manual control"
  else
    printf '  %-24s receiver present; /joy stale\n' "manual control"
    ready=false
  fi
else
  printf '  %-24s joystick receiver absent\n' "manual control"
  ready=false
fi

for specification in \
    "service /patrolbot/charge_release" \
    "service /patrolbot/motor_enable" \
    "service /patrolbot/initialize_dock_pose" \
    "action /patrolbot/undock"; do
  read -r kind name <<<"$specification"
  if ros_name_exists "$kind" "$name"; then
    printf '  %-24s advertised\n' "$name"
  else
    printf '  %-24s missing\n' "$name"
    ready=false
  fi
done

readiness_publishers=$(topic_publisher_count /patrolbot/dock_readiness) || \
  readiness_publishers=unknown
if [[ "$readiness_publishers" == 1 ]]; then
  printf '  %-24s one publisher\n' "/patrolbot/dock_readiness"
else
  printf '  %-24s %s publishers (expected one)\n' \
    "/patrolbot/dock_readiness" "$readiness_publishers"
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
