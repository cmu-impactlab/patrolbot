# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO

RUN --mount=type=tmpfs,target=/var/cache/apt/archives \
    apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      cmake \
      build-essential \
      ros-${ROS_DISTRO}-nav2-bringup \
      ros-${ROS_DISTRO}-slam-toolbox \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-twist-mux \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-rclcpp-components \
      ros-${ROS_DISTRO}-urg-node \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/patrolbot_ws

# rosaria2 is versioned in the monorepo as a legacy fallback, but is deliberately
# excluded from this image because it conflicts with patrolbot_bridge.
COPY ros2_ws/src/patrolbot_interfaces src/patrolbot_interfaces
COPY ros2_ws/src/patrolbot_bridge src/patrolbot_bridge
COPY ros2_ws/src/patrolbot_dock_manager src/patrolbot_dock_manager
COPY ros2_ws/src/patrolbot_docking_plugins src/patrolbot_docking_plugins
COPY ros2_ws/src/patrolbot-launch src/patrolbot-launch
COPY ros2_ws/src/patrolbot_navigation src/patrolbot_navigation
# Retain the SBC protocol source as a static-test fixture. It is not executed
# in the Pi image; navigation's safety-contract test verifies that dock release
# cannot set velocity or enable motors as a side effect.
COPY hardware/patrolbot_hw_server hardware/patrolbot_hw_server
COPY docker/Dockerfile docker/docker-compose.yml docker/status.sh \
  docker/source-tree-digest.sh docker/capture-nav-goal.sh docker/
COPY --chmod=0755 docker/source-tree-digest.sh /usr/local/bin/patrolbot-source-tree-digest

ARG PATROLBOT_SOURCE_SHA256=unknown
RUN actual=$(/usr/local/bin/patrolbot-source-tree-digest /opt/patrolbot_ws) && \
    if [ "$PATROLBOT_SOURCE_SHA256" != "unknown" ] && \
       [ "$actual" != "$PATROLBOT_SOURCE_SHA256" ]; then \
      printf 'source digest mismatch: expected %s, got %s\n' \
        "$PATROLBOT_SOURCE_SHA256" "$actual" >&2; \
      exit 1; \
    fi && \
    printf '%s\n' "$actual" > /opt/patrolbot_ws/PATROLBOT_SOURCE_SHA256

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install \
      --packages-select patrolbot_interfaces patrolbot_bridge \
        patrolbot_dock_manager patrolbot_docking_plugins patrolbot-launch \
        patrolbot_navigation

COPY --chmod=0755 docker/healthcheck.py /usr/local/bin/patrolbot-healthcheck

RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
      'source /opt/patrolbot_ws/install/setup.bash' \
      'exec "$@"' \
      > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

ARG PATROLBOT_REVISION=unknown
LABEL org.opencontainers.image.title="PatrolBot ROS 2 stack" \
      org.opencontainers.image.source="https://github.com/cmu-impactlab/patrolbot" \
      org.opencontainers.image.revision="${PATROLBOT_REVISION}" \
      io.patrolbot.source-tree-sha256="${PATROLBOT_SOURCE_SHA256}"
