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

if [[ "$all_healthy" != true ]]; then
  printf '\nOVERALL=unhealthy reason=container-liveness\n'
  exit 1
fi

if ! timeout 2 bash -c "exec 3<>/dev/tcp/$SBC_HOST/$SBC_PORT" 2>/dev/null; then
  printf '\nSBC readiness\n'
  printf '  tcp://%s:%s unavailable\n' "$SBC_HOST" "$SBC_PORT"
  printf '  telemetry and odom->base_link checks skipped\n'
  printf '\nOVERALL=degraded reason=sbc-unavailable\n'
  exit 2
fi

ros_exec() {
  local container=$1
  shift
  timeout 15 docker exec "$container" bash -lc \
    "source /opt/ros/\${ROS_DISTRO}/setup.bash; source /opt/patrolbot_ws/install/setup.bash; $*"
}

topic_is_fresh() {
  local topic=$1
  local output
  output=$(ros_exec patrolbot-bridge \
    "timeout 8 ros2 topic hz --window 3 '$topic' 2>&1 || true") || return 1
  grep -q 'average rate:' <<<"$output"
}

tf_is_ready() {
  local parent=$1 child=$2 output
  output=$(ros_exec patrolbot-navigation \
    "timeout 8 ros2 run tf2_ros tf2_echo '$parent' '$child' 2>&1 || true") || return 1
  grep -q 'Translation:' <<<"$output"
}

lifecycle_is_active() {
  local container=$1 node=$2 output
  output=$(ros_exec "$container" \
    "ros2 daemon stop >/dev/null 2>&1 || true; sleep 1; ros2 lifecycle get '$node' 2>&1") ||
    return 1
  grep -q '^active ' <<<"$output"
}

printf '\nSBC and ROS readiness\n'
printf '  tcp://%s:%s reachable\n' "$SBC_HOST" "$SBC_PORT"

ready=true
for topic in /odom /scan; do
  if topic_is_fresh "$topic"; then
    printf '  %-24s fresh\n' "$topic"
  else
    printf '  %-24s unavailable/stale\n' "$topic"
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
