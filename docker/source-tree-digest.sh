#!/usr/bin/env bash
# Deterministic digest of all source that is compiled into the PatrolBot image.
set -euo pipefail

ROOT=${1:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)}

# The repository stores packages under ros2_ws/src, while the built image uses
# the standard colcon workspace path src. Hash canonical repository-relative
# names in both layouts so the build label can be compared to the deployed
# source tree without a false mismatch caused only by COPY destinations.
if [[ -d "$ROOT/ros2_ws/src" ]]; then
  PACKAGES_ROOT="$ROOT/ros2_ws/src"
elif [[ -d "$ROOT/src" ]]; then
  PACKAGES_ROOT="$ROOT/src"
else
  printf 'missing ROS package root under %s\n' "$ROOT" >&2
  exit 2
fi

package_names=(
  patrolbot_interfaces
  patrolbot_bridge
  patrolbot_dock_manager
  patrolbot_docking_plugins
  patrolbot-launch
  patrolbot_navigation
)
paths=()
for package in "${package_names[@]}"; do
  path="$PACKAGES_ROOT/$package"
  if [[ ! -d "$path" ]]; then
    printf 'missing source path: %s\n' "$path" >&2
    exit 2
  fi
  paths+=("$path")
done
hardware_path="$ROOT/hardware/patrolbot_hw_server"
if [[ ! -d "$hardware_path" ]]; then
  printf 'missing source path: %s\n' "$hardware_path" >&2
  exit 2
fi
paths+=("$hardware_path")

LC_ALL=C find "${paths[@]}" -type f \
  ! -path '*/__pycache__/*' \
  ! -path '*/.pytest_cache/*' \
  ! -path "$ROOT/*/backups/*" \
  ! -path '*/build/*' \
  ! -path '*/install/*' \
  ! -path '*/log/*' \
  ! -name '*.pyc' \
  ! -name '*.orig' \
  ! -name '*.rej' \
  ! -name '*.bak' \
  ! -name '*.bak-*' \
  ! -name '*.bak.*' \
  ! -name '*.precontroller.bak' \
  ! -name '*~' \
  ! -name 'patrolbot_server' \
  ! -name 'patrolbot_commission_probe' \
  ! -path '*/tests/protocol_test' \
  ! -path '*/tests/undock_test' \
  ! -path '*/tests/scan_timing_test' \
  ! -path '*/tests/lidar_power_test' \
  ! -path '*/tests/encoder_capture_test' \
  -print0 \
  | LC_ALL=C sort -z \
  | while IFS= read -r -d '' file; do
      if [[ "$file" == "$PACKAGES_ROOT/"* ]]; then
        canonical="ros2_ws/src/${file#"$PACKAGES_ROOT/"}"
      else
        canonical="${file#"$ROOT/"}"
      fi
      hash=$(sha256sum "$file")
      printf '%s  %s\n' "${hash%% *}" "$canonical"
    done \
  | sha256sum \
  | awk '{print $1}'
