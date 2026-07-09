# syntax=docker/dockerfile:1
ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base
ARG ROS_DISTRO
ARG PATROLBOT_REVISION=unknown

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
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/patrolbot_ws

# rosaria2 is versioned in the monorepo as a legacy fallback, but is deliberately
# excluded from this image because it conflicts with patrolbot_bridge.
COPY ros2_ws/src/patrolbot_bridge src/patrolbot_bridge
COPY ros2_ws/src/patrolbot-launch src/patrolbot-launch
COPY ros2_ws/src/patrolbot_navigation src/patrolbot_navigation

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install \
      --packages-select patrolbot_bridge patrolbot-launch patrolbot_navigation

COPY --chmod=0755 docker/healthcheck.py /usr/local/bin/patrolbot-healthcheck

RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
      'source /opt/patrolbot_ws/install/setup.bash' \
      'exec "$@"' \
      > /entrypoint.sh && chmod +x /entrypoint.sh

LABEL org.opencontainers.image.title="PatrolBot ROS 2 stack" \
      org.opencontainers.image.source="https://github.com/cmu-impactlab/patrolbot" \
      org.opencontainers.image.revision="${PATROLBOT_REVISION}"

ENTRYPOINT ["/entrypoint.sh"]
