#!/usr/bin/env bash
# Bounded, read-only capture for a Nav2 goal that moves briefly and stops.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_ROOT=$(cd -- "$SCRIPT_DIR/.." && pwd)
DURATION=${1:-20}

if [[ ! "$DURATION" =~ ^[0-9]+$ ]] || (( DURATION < 1 || DURATION > 120 )); then
  printf 'usage: %s [duration-seconds: 1-120]\n' "$0" >&2
  exit 2
fi

for container in patrolbot-navigation patrolbot-bringup patrolbot-bridge; do
  state=$(docker inspect "$container" --format '{{.State.Status}}' 2>/dev/null || true)
  if [[ "$state" != running ]]; then
    printf '%s is not running\n' "$container" >&2
    exit 1
  fi
done

for container in patrolbot-navigation patrolbot-bringup patrolbot-bridge; do
  if docker exec "$container" ps -eo args 2>/dev/null |
      grep -q '[r]os2 topic hz'; then
    printf 'refusing capture: unattended ros2 topic hz exists in %s\n' \
      "$container" >&2
    exit 1
  fi
done

stamp=$(date -u +%Y%m%dT%H%M%SZ)
output_dir="$REPOSITORY_ROOT/logs/nav-goal-captures/$stamp"
mkdir -p "$output_dir"
started_at=$(date -Is)

{
  printf 'started_at=%s\n' "$started_at"
  printf 'duration_s=%s\n' "$DURATION"
  docker ps --format '{{.Names}} {{.Status}} {{.Image}}'
  docker inspect patrolbot-navigation --format \
    'revision={{index .Config.Labels "org.opencontainers.image.revision"}} source_digest={{index .Config.Labels "io.patrolbot.source-tree-sha256"}}'
} >"$output_dir/metadata.txt"

docker exec patrolbot-navigation timeout --signal=INT --kill-after=2 15 \
  bash -lc '
    source /opt/ros/$ROS_DISTRO/setup.bash
    source /opt/patrolbot_ws/install/setup.bash
    for topic in \
      /input/safety_controller /input/joy /input/navi \
      /cmd_vel_out /cmd_vel /scan /sonar /odom /amcl_pose \
      /patrolbot/base_state /navigate_to_pose/_action/status; do
      printf "\n### %s\n" "$topic"
      ros2 topic info -v "$topic" || true
    done
    printf "\n### lifecycle\n"
    ros2 lifecycle get /controller_server || true
    ros2 lifecycle get /bt_navigator || true
    ros2 lifecycle get /velocity_smoother || true
    ros2 lifecycle get /collision_monitor || true
  ' >"$output_dir/endpoints.txt" 2>&1

timeout_s=$((DURATION + 5))
docker exec patrolbot-navigation \
  timeout --signal=INT --kill-after=2 "$timeout_s" \
  bash -lc "
    source /opt/ros/\$ROS_DISTRO/setup.bash
    source /opt/patrolbot_ws/install/setup.bash
    ros2 run patrolbot_navigation capture_nav_command_path.py \
      --duration '$DURATION'
  " >"$output_dir/command-path.jsonl" 2>"$output_dir/capture.stderr"

for container in patrolbot-navigation patrolbot-bringup patrolbot-bridge; do
  docker logs --since "$started_at" "$container" \
    >"$output_dir/$container.log" 2>&1 || true
done

if find "$output_dir" -type f -size +0c | grep -q .; then
  printf 'capture=%s\n' "$output_dir"
else
  printf 'capture produced no evidence: %s\n' "$output_dir" >&2
  exit 1
fi
