#!/usr/bin/env bash
# Read-only PatrolBot liveness/readiness report.
set -u

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
COMPOSE=(docker compose -f "$SCRIPT_DIR/docker-compose.yml" --env-file "$SCRIPT_DIR/.env")
CONTAINERS=(patrolbot-bringup patrolbot-bridge patrolbot-navigation)
SBC_HOST=${PATROLBOT_SBC_HOST:-10.0.0.1}
SBC_PORT=${PATROLBOT_SBC_PORT:-7272}
MACHINE_CHECKS=()

record_check() {
  local scope=$1 name=$2 status=$3 evidence=$4
  evidence=${evidence//$'\n'/ }
  evidence=${evidence//|//}
  MACHINE_CHECKS+=("$scope|$name|$status|$evidence")
}

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
    record_check container "container.${container#patrolbot-}" fail missing
    all_healthy=false
    continue
  fi
  printf '  %-24s %s\n' "$container" "$state"
  if [[ "$state" == *"status=running health=healthy"* ]]; then
    record_check container "container.${container#patrolbot-}" pass "$state"
  else
    record_check container "container.${container#patrolbot-}" fail "$state"
    all_healthy=false
  fi
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
discovery_expected=""
for container in "${CONTAINERS[@]}"; do
  discovery_range=$(docker inspect "$container" \
    --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null \
    | sed -n 's/^ROS_AUTOMATIC_DISCOVERY_RANGE=//p' | head -n 1)
  printf '  %-24s %s\n' "$container discovery" \
    "${discovery_range:-unset}"
  if [[ "$discovery_range" != LOCALHOST && "$discovery_range" != SUBNET ]]; then
    discovery_ok=false
  elif [[ -z "$discovery_expected" ]]; then
    discovery_expected=$discovery_range
  elif [[ "$discovery_range" != "$discovery_expected" ]]; then
    discovery_ok=false
  fi
done

bridge_publication_mode=$(docker inspect patrolbot-bridge \
  --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null \
  | sed -n 's/^RMW_FASTRTPS_PUBLICATION_MODE=//p' | head -n 1)
printf '  %-24s %s\n' "bridge DDS publication" \
  "${bridge_publication_mode:-unset}"

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

interface_type_hash() {
  # The RIHS01 hash rosidl generated for one interface, read out of the image's
  # own installed type description rather than off the DDS graph. A subscriber
  # whose hash disagrees is never matched, so it does not appear on the topic at
  # all -- which is exactly why the DDS graph cannot be used to detect this.
  local container=$1 kind=$2 name=$3
  docker exec "$container" timeout --signal=INT --kill-after=2 10 python3 -c '
import json
import sys

kind, name = sys.argv[1], sys.argv[2]
path = ("/opt/patrolbot_ws/install/patrolbot_interfaces/share/"
        "patrolbot_interfaces/{}/{}.json".format(kind, name))
with open(path) as handle:
    hashes = json.load(handle)["type_hashes"]
wanted = "patrolbot_interfaces/{}/{}".format(kind, name)
for entry in hashes:
    if entry["type_name"] == wanted:
        print(entry["hash_string"])
        break
' "$kind" "$name" 2>/dev/null
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
ready=$all_healthy

if [[ "$source_digest_ok" != true ]]; then
  printf '  %-24s mismatch (image must match deployed source)\n' \
    "source provenance"
  ready=false
  record_check pi source.provenance fail \
    "image=${image_source_digest:-unset} deployed=${deployed_source_digest:-unavailable}"
else
  printf '  %-24s verified\n' "source provenance"
  record_check pi source.provenance pass "$image_source_digest"
fi

if [[ "$discovery_ok" != true ]]; then
  printf '  %-24s invalid (expected one consistent LOCALHOST or SUBNET policy)\n' \
    "DDS discovery scope"
  ready=false
  record_check pi dds.discovery fail "inconsistent or invalid"
elif [[ "$discovery_expected" == SUBNET ]]; then
  printf '  %-24s SUBNET (full LAN graph; unauthenticated)\n' \
    "DDS discovery scope"
  record_check pi dds.discovery pass "SUBNET unauthenticated"
else
  printf '  %-24s LOCALHOST only\n' "DDS discovery scope"
  record_check pi dds.discovery pass "LOCALHOST"
fi

# PB-AUDIT-025. The source digest above only covers the core stack, so a derived
# image (the dashboard bridge) could lag behind with an older
# `patrolbot_interfaces`. ROS 2 then refuses to match its subscription against
# the live publisher because the message type hashes differ -- silently, with no
# error on either side. That cost a day of dashboard dock telemetry. Every
# container carrying the interfaces must agree on the hash of the messages the
# dock path depends on.
for interface in msg/BaseState msg/DockReadiness; do
  interface_kind=${interface%%/*}
  interface_name=${interface##*/}
  reference_hash=""
  reference_container=""
  interface_hash_ok=true
  for container in "${CONTAINERS[@]}" patrolbot-web-bridge; do
    docker inspect "$container" >/dev/null 2>&1 || continue
    container_hash=$(interface_type_hash \
      "$container" "$interface_kind" "$interface_name") || container_hash=""
    if [[ -z "$container_hash" ]]; then
      printf '  %-24s %s: hash unreadable\n' "$interface_name types" "$container"
      interface_hash_ok=false
      continue
    fi
    if [[ -z "$reference_hash" ]]; then
      reference_hash=$container_hash
      reference_container=$container
    elif [[ "$container_hash" != "$reference_hash" ]]; then
      printf '  %-24s %s disagrees with %s (%s vs %s)\n' \
        "$interface_name types" "$container" "$reference_container" \
        "${container_hash:0:18}" "${reference_hash:0:18}"
      interface_hash_ok=false
    fi
  done
  if [[ "$interface_hash_ok" == true && -n "$reference_hash" ]]; then
    printf '  %-24s one hash (%s)\n' \
      "$interface_name types" "${reference_hash:0:18}"
    record_check pi "interface.$interface_name" pass "$reference_hash"
  else
    record_check pi "interface.$interface_name" fail "hash mismatch or unreadable"
    ready=false
  fi
done

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
  record_check pi diagnostics.unattended-monitors fail "$monitor_processes"
else
  printf '  %-24s none\n' "unattended topic monitors"
  record_check pi diagnostics.unattended-monitors pass none
fi

if [[ "$bridge_publication_mode" != ASYNCHRONOUS ]]; then
  printf '  %-24s %s (expected ASYNCHRONOUS)\n' \
    "bridge DDS publication" "${bridge_publication_mode:-unset}"
  ready=false
  record_check pi dds.bridge-publication fail "${bridge_publication_mode:-unset}"
else
  record_check pi dds.bridge-publication pass ASYNCHRONOUS
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
    "/cmd_vel_out SUBSCRIPTION patrolbot_controller_feedback,patrolbot_safety_watchdog,teleop_velocity_smoother" \
    "/cmd_vel PUBLISHER teleop_velocity_smoother" \
    "/cmd_vel SUBSCRIPTION patrolbot_bridge"; do
  read -r topic endpoint expected <<<"$specification"
  actual=$(topic_endpoint_nodes "$topic" "$endpoint") || actual=unknown
  if [[ "$actual" == "${expected:-}" ]]; then
    printf '  %-24s %s\n' "$topic $endpoint" "${actual:-none}"
    record_check pi "endpoint.$topic.$endpoint" pass "${actual:-none}"
  else
    printf '  %-24s %s (expected %s)\n' \
      "$topic $endpoint" "${actual:-none}" "${expected:-none}"
    ready=false
    record_check pi "endpoint.$topic.$endpoint" fail \
      "actual=${actual:-none} expected=${expected:-none}"
  fi
done

if ros_name_exists node /docking_server; then
  printf '  %-24s PRESENT (must remain disabled)\n' "/docking_server"
  ready=false
  record_check pi node.docking_server fail present
else
  printf '  %-24s absent\n' "/docking_server"
  record_check pi node.docking_server pass absent
fi

if ros_name_exists topic /scan_unfiltered_candidate; then
  printf '  %-24s PRESENT (must remain uncommissioned)\n' \
    "/scan_unfiltered_candidate"
  ready=false
  record_check pi topic.scan_unfiltered_candidate fail present
else
  printf '  %-24s absent/uncommissioned\n' \
    "/scan_unfiltered_candidate"
  record_check pi topic.scan_unfiltered_candidate pass absent
fi

smoother_max=$(ros_exec patrolbot-bringup \
  "ros2 param get /teleop_velocity_smoother max_velocity 2>&1" || true)
if [[ "$smoother_max" == *"[0.3, 0.0, 0.7]"* ]]; then
  printf '  %-24s [0.30, 0.0, 0.7]\n' "teleop smoother max"
  record_check pi parameter.teleop-smoother-max pass "[0.3,0.0,0.7]"
else
  printf '  %-24s %s (unexpected)\n' "teleop smoother max" \
    "${smoother_max:-unavailable}"
  ready=false
  record_check pi parameter.teleop-smoother-max fail \
    "${smoother_max:-unavailable}"
fi

battery_stop=$(ros_exec patrolbot-navigation \
  "ros2 param get /patrolbot_safety_watchdog battery_stop_voltage 2>&1" || true)
bumper_enforcement=$(ros_exec patrolbot-navigation \
  "ros2 param get /patrolbot_safety_watchdog bumper_enforcement_enabled 2>&1" || true)
if [[ "$battery_stop" == *"23.0"* ]]; then
  printf '  %-24s 23.0 V\n' "battery motion stop"
  record_check pi parameter.battery-stop pass "23.0 V"
else
  printf '  %-24s %s (expected 23.0 V)\n' "battery motion stop" \
    "${battery_stop:-unavailable}"
  ready=false
  record_check pi parameter.battery-stop fail "${battery_stop:-unavailable}"
fi
if [[ "$bumper_enforcement" == *"False"* ]]; then
  printf '  %-24s disabled/uncommissioned\n' "bumper enforcement"
  record_check pi parameter.bumper-enforcement pass disabled
else
  printf '  %-24s %s (requires physical mapping review)\n' \
    "bumper enforcement" "${bumper_enforcement:-unavailable}"
  ready=false
  record_check pi parameter.bumper-enforcement fail \
    "${bumper_enforcement:-unavailable}"
fi

if docker exec patrolbot-bringup test -e /dev/input/js0 2>/dev/null; then
  if topic_is_fresh /joy; then
    printf '  %-24s receiver present; /joy fresh\n' "manual control"
    record_check pi topic.joy pass "receiver present and fresh"
  else
    printf '  %-24s receiver present; /joy stale\n' "manual control"
    ready=false
    record_check pi topic.joy fail "receiver present but stale"
  fi
else
  printf '  %-24s joystick receiver absent\n' "manual control"
  ready=false
  record_check pi topic.joy fail "receiver absent"
fi

for specification in \
    "service /patrolbot/charge_release" \
    "service /patrolbot/motor_enable" \
    "service /patrolbot/initialize_dock_pose" \
    "service /request_nomotion_update" \
    "action /patrolbot/undock" \
    "action /navigate_to_pose" \
    "action /navigate_through_poses" \
    "action /compute_path_to_pose" \
    "action /follow_path" \
    "action /spin" \
    "action /backup"; do
  read -r kind name <<<"$specification"
  if ros_name_exists "$kind" "$name"; then
    printf '  %-24s advertised\n' "$name"
    record_check pi "$kind.$name" pass advertised
  else
    printf '  %-24s missing\n' "$name"
    ready=false
    record_check pi "$kind.$name" fail missing
  fi
done

readiness_publishers=$(topic_publisher_count /patrolbot/dock_readiness) || \
  readiness_publishers=unknown
if [[ "$readiness_publishers" == 1 ]]; then
  printf '  %-24s one publisher\n' "/patrolbot/dock_readiness"
  record_check pi topic.dock-readiness.publishers pass one
else
  printf '  %-24s %s publishers (expected one)\n' \
    "/patrolbot/dock_readiness" "$readiness_publishers"
  ready=false
  record_check pi topic.dock-readiness.publishers fail \
    "${readiness_publishers} expected one"
fi

if topic_is_fresh /odom; then
  printf '  %-24s fresh (SBC link up)\n' "/odom"
  sbc_available=true
  record_check sensor topic.odom.fresh pass "SBC link up"
else
  printf '  %-24s unavailable/stale\n' "/odom"
  printf '  SBC link unavailable (bridge not receiving /odom)\n'
  printf '  sensor freshness and dynamic TF are degraded; Pi checks continue\n'
  sbc_available=false
  record_check sensor topic.odom.fresh degraded "SBC unavailable"
fi

if topic_is_fresh /scan; then
  printf '  %-24s fresh\n' "/scan"
  record_check sensor topic.scan.fresh pass fresh
else
  printf '  %-24s unavailable/stale\n' "/scan"
  if [[ "$sbc_available" == true ]]; then
    record_check sensor topic.scan.fresh fail "stale while odom is fresh"
  else
    record_check sensor topic.scan.fresh degraded "SBC unavailable"
  fi
fi

# One bridge publisher per hardware topic is a deployment invariant. This
# catches an accidentally re-enabled Pi 4 rollback stack before duplicate
# odom/scan streams can corrupt TF, costmaps, and navigation.
for topic in /odom /scan; do
  count=$(topic_publisher_count "$topic") || count=unknown
  if [[ "$count" == 1 ]]; then
    printf '  %-24s one\n' "$topic publisher"
    record_check sensor "topic.$topic.publishers" pass one
  else
    printf '  %-24s %s (expected one)\n' "$topic publishers" "$count"
    ready=false
    record_check sensor "topic.$topic.publishers" fail \
      "$count expected one"
  fi
done

for topic in /scan /sonar; do
  reliability=$(topic_publisher_reliability "$topic") || reliability=unknown
  if [[ "$reliability" == BEST_EFFORT ]]; then
    printf '  %-24s BEST_EFFORT\n' "$topic publisher QoS"
    record_check sensor "topic.$topic.qos" pass BEST_EFFORT
  else
    printf '  %-24s %s (expected BEST_EFFORT)\n' \
      "$topic publisher QoS" "$reliability"
    ready=false
    record_check sensor "topic.$topic.qos" fail \
      "$reliability expected BEST_EFFORT"
  fi
done

printf '\nPi node and lifecycle readiness\n'

for specification in \
    "patrolbot-bringup /teleop_velocity_smoother" \
    "patrolbot-navigation /map_server" \
    "patrolbot-navigation /amcl" \
    "patrolbot-navigation /controller_server" \
    "patrolbot-navigation /smoother_server" \
    "patrolbot-navigation /planner_server" \
    "patrolbot-navigation /route_server" \
    "patrolbot-navigation /behavior_server" \
    "patrolbot-navigation /velocity_smoother" \
    "patrolbot-navigation /collision_monitor" \
    "patrolbot-navigation /bt_navigator" \
    "patrolbot-navigation /waypoint_follower"; do
  read -r container node <<<"$specification"
  if lifecycle_is_active "$container" "$node"; then
    printf '  %-24s active\n' "$node"
    record_check pi "lifecycle.$node" pass active
  else
    printf '  %-24s inactive/unavailable\n' "$node"
    record_check pi "lifecycle.$node" fail inactive
    ready=false
  fi
done

for node in \
    /lifecycle_manager_script \
    /lifecycle_manager_localization \
    /lifecycle_manager_navigation \
    /nav_command_mux \
    /cmd_vel_mux \
    /patrolbot_safety_watchdog \
    /patrolbot_dock_manager \
    /patrolbot_joy_teleop \
    /laser_static_tf; do
  if ros_name_exists node "$node"; then
    printf '  %-24s present\n' "$node"
    record_check pi "node.$node" pass present
  else
    printf '  %-24s missing\n' "$node"
    record_check pi "node.$node" fail missing
    ready=false
  fi
done

printf '\nTF readiness\n'

for frames in "odom base_link" "map odom" "base_link laser_frame"; do
  read -r parent child <<<"$frames"
  if tf_is_ready "$parent" "$child"; then
    printf '  %-24s ready\n' "$parent->$child"
    if [[ "$parent" == base_link ]]; then
      record_check pi "tf.$parent-$child" pass ready
    else
      record_check sensor "tf.$parent-$child" pass ready
    fi
  else
    printf '  %-24s unavailable\n' "$parent->$child"
    if [[ "$parent" == base_link ]]; then
      record_check pi "tf.$parent-$child" fail unavailable
      ready=false
    elif [[ "$sbc_available" == true ]]; then
      record_check sensor "tf.$parent-$child" fail unavailable
    else
      record_check sensor "tf.$parent-$child" skipped \
        "requires SBC sensor data"
    fi
  fi
done

# Advisory retained-log evidence, independent of current readiness. Bound both
# the requested window and line count; never infer current health from silence.
if [[ -f "$SCRIPT_DIR/nav2-log-health.py" ]]; then
  printf '\nNav2 warning history (last hour, at most 10000 lines)\n'
  if nav2_log_summary=$(
      set -o pipefail
      timeout 10 docker logs --since 1h --tail 10000 \
      patrolbot-navigation 2>&1 | python3 "$SCRIPT_DIR/nav2-log-health.py"); then
    printf 'NAV2_LOG_HEALTH_JSON=%s\n' "$nav2_log_summary"
  else
    printf 'NAV2_LOG_HEALTH_JSON={"error":"log collection unavailable"}\n'
  fi
fi

printf '\nMachine-readable readiness\n'
python3 "$SCRIPT_DIR/status_report.py" "${MACHINE_CHECKS[@]}"
exit $?
