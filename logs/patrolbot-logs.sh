#!/usr/bin/env bash
# View PatrolBot container output or persisted ROS logs with health context.
set -Eeuo pipefail

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)
COMPOSE_FILE="$REPO_ROOT/docker/docker-compose.yml"
ENV_FILE="$REPO_ROOT/docker/.env"
STATUS_SCRIPT="$REPO_ROOT/docker/status.sh"

ALL_MODULES=(bringup bridge navigation)
MODULES=()
SOURCE=docker
FOLLOW=true
TAIL_LINES=200
SINCE=
SHOW_HEALTH=true
RUN_READINESS=false
HEALTH_ONLY=false
TIMESTAMPS=true
NO_COLOR=false
HEALTH_OK=true
READINESS_RESULT=0

usage() {
  cat <<'EOF'
Usage: logs/patrolbot-logs.sh [MODULE ...] [OPTIONS]

View logs for bringup, bridge, navigation, or all three services together.
With no module or option, the script shows lightweight container health and
follows the last 200 lines from all services.

Modules:
  all                         Select all three modules (default)
  bringup                     twist_mux and velocity smoother
  bridge                      SBC-to-ROS bridge
  navigation                  Nav2, joystick, safety, and laser TF

Options:
  -m, --module NAME           Select a module; repeat to select several
  -n, --tail LINES            Lines per service/file (default: 200; or "all")
      --since DURATION        Docker logs since a value such as 10m or 2h
  -f, --follow                Continue streaming logs (default)
      --no-follow             Print the current log snapshot and exit
      --source docker|ros     Docker output or persisted ROS log files
      --readiness             Run the full ROS/SBC readiness check first
      --health-only           Show container health without displaying logs
      --no-health             Do not print the lightweight health summary
      --no-timestamps         Hide Docker timestamps
      --no-color              Disable Docker Compose log colors
  -h, --help                  Show this help

Examples:
  logs/patrolbot-logs.sh
  logs/patrolbot-logs.sh bridge --tail 500
  logs/patrolbot-logs.sh bringup navigation --since 15m --no-follow
  logs/patrolbot-logs.sh all --source ros --no-follow
  logs/patrolbot-logs.sh --health-only --readiness
EOF
}

die() {
  printf 'patrolbot-logs: %s\n' "$*" >&2
  exit 2
}

add_module() {
  local requested=$1 module existing
  if [[ "$requested" == all ]]; then
    for module in "${ALL_MODULES[@]}"; do
      add_module "$module"
    done
    return
  fi

  case "$requested" in
    bringup|bridge|navigation) ;;
    *) die "unknown module '$requested' (use bringup, bridge, navigation, or all)" ;;
  esac

  for existing in "${MODULES[@]}"; do
    [[ "$existing" == "$requested" ]] && return
  done
  MODULES+=("$requested")
}

require_value() {
  local option=$1 remaining=$2
  (( remaining >= 2 )) || die "$option requires a value"
}

while (( $# > 0 )); do
  case "$1" in
    all|bringup|bridge|navigation)
      add_module "$1"
      shift
      ;;
    -m|--module)
      require_value "$1" "$#"
      add_module "$2"
      shift 2
      ;;
    --module=*)
      add_module "${1#*=}"
      shift
      ;;
    -n|--tail)
      require_value "$1" "$#"
      TAIL_LINES=$2
      shift 2
      ;;
    --tail=*)
      TAIL_LINES=${1#*=}
      shift
      ;;
    --since)
      require_value "$1" "$#"
      SINCE=$2
      shift 2
      ;;
    --since=*)
      SINCE=${1#*=}
      shift
      ;;
    -f|--follow)
      FOLLOW=true
      shift
      ;;
    --no-follow)
      FOLLOW=false
      shift
      ;;
    --source)
      require_value "$1" "$#"
      SOURCE=$2
      shift 2
      ;;
    --source=*)
      SOURCE=${1#*=}
      shift
      ;;
    --readiness)
      RUN_READINESS=true
      shift
      ;;
    --health-only)
      HEALTH_ONLY=true
      shift
      ;;
    --no-health)
      SHOW_HEALTH=false
      shift
      ;;
    --no-timestamps)
      TIMESTAMPS=false
      shift
      ;;
    --no-color)
      NO_COLOR=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      while (( $# > 0 )); do
        add_module "$1"
        shift
      done
      ;;
    -*) die "unknown option '$1'" ;;
    *) die "unknown module '$1'" ;;
  esac
done

[[ "$TAIL_LINES" == all || "$TAIL_LINES" =~ ^[0-9]+$ ]] ||
  die "--tail must be a non-negative number or 'all'"
case "$SOURCE" in
  docker|ros) ;;
  *) die "--source must be 'docker' or 'ros'" ;;
esac
[[ -z "$SINCE" || "$SOURCE" == docker ]] ||
  die "--since is available only with --source docker"
(( ${#MODULES[@]} > 0 )) || add_module all

command -v docker >/dev/null 2>&1 || die "docker is not installed or not in PATH"
[[ -f "$COMPOSE_FILE" ]] || die "Compose file not found: $COMPOSE_FILE"

container_for() {
  printf 'patrolbot-%s\n' "$1"
}

show_health() {
  local module container state revision
  printf 'PatrolBot container health\n'
  for module in "${MODULES[@]}"; do
    container=$(container_for "$module")
    if state=$(docker inspect "$container" \
        --format 'status={{.State.Status}} health={{if .State.Health}}{{.State.Health.Status}}{{else}}none{{end}} restarts={{.RestartCount}} image={{.Config.Image}}' \
        2>/dev/null); then
      printf '  %-12s %s\n' "$module" "$state"
      [[ "$state" == *"status=running health=healthy"* ]] || HEALTH_OK=false
    else
      printf '  %-12s missing\n' "$module"
      HEALTH_OK=false
    fi
  done

  revision=$(docker inspect patrolbot-navigation \
    --format '{{index .Config.Labels "org.opencontainers.image.revision"}}' \
    2>/dev/null || true)
  printf '  %-12s %s\n' revision "${revision:-unknown}"
}

if [[ "$SHOW_HEALTH" == true ]]; then
  show_health
fi

if [[ "$RUN_READINESS" == true ]]; then
  printf '\nFull PatrolBot readiness\n'
  if "$STATUS_SCRIPT"; then
    READINESS_RESULT=0
  else
    READINESS_RESULT=$?
    printf 'Readiness is not complete; continuing with the requested log view.\n' >&2
  fi
fi

if [[ "$HEALTH_ONLY" == true ]]; then
  (( READINESS_RESULT == 0 )) || exit "$READINESS_RESULT"
  [[ "$HEALTH_OK" == true ]] || exit 1
  exit 0
fi

printf '\nViewing %s logs for: %s\n' "$SOURCE" "${MODULES[*]}"
if [[ "$FOLLOW" == true ]]; then
  printf 'Press Ctrl-C to stop.\n\n'
else
  printf '\n'
fi

if [[ "$SOURCE" == docker ]]; then
  COMPOSE=(docker compose -f "$COMPOSE_FILE")
  [[ ! -f "$ENV_FILE" ]] || COMPOSE+=(--env-file "$ENV_FILE")
  LOG_ARGS=(logs "--tail=$TAIL_LINES")
  [[ -z "$SINCE" ]] || LOG_ARGS+=(--since "$SINCE")
  [[ "$FOLLOW" == false ]] || LOG_ARGS+=(--follow)
  [[ "$TIMESTAMPS" == false ]] || LOG_ARGS+=(--timestamps)
  [[ "$NO_COLOR" == false ]] || LOG_ARGS+=(--no-color)
  LOG_ARGS+=("${MODULES[@]}")
  exec "${COMPOSE[@]}" "${LOG_ARGS[@]}"
fi

ROS_FILES=()
for module in "${MODULES[@]}"; do
  module_dir="$SCRIPT_DIR/$module"
  [[ -d "$module_dir" ]] || continue
  while IFS= read -r -d '' log_file; do
    ROS_FILES+=("$log_file")
  done < <(find "$module_dir" -type f -name '*.log' -print0 | sort -z)
done

(( ${#ROS_FILES[@]} > 0 )) || die "no persisted ROS .log files found for: ${MODULES[*]}"
ROS_TAIL=$TAIL_LINES
[[ "$ROS_TAIL" != all ]] || ROS_TAIL=+1
if [[ "$FOLLOW" == true ]]; then
  exec tail -n "$ROS_TAIL" -F "${ROS_FILES[@]}"
fi
exec tail -n "$ROS_TAIL" "${ROS_FILES[@]}"
