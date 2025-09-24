#!/usr/bin/env bash
set -e
_source(){ [ -f "$1" ] && source "$1"; }

# ROS base + optional overlays
_source /opt/ros/humble/setup.bash
_source /opt/link_attacher/install/setup.bash

# Repo workspace (host or container)
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_source "${REPO_ROOT}/install/setup.bash"

# Defaults (override-able)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-130}"
export LC_ALL="${LC_ALL:-C}"
export LANG="${LANG:-C}"

# IGN path from ur_description 
if command -v ros2 >/dev/null 2>&1; then
  UR_PREFIX="$(ros2 pkg prefix ur_description 2>/dev/null || true)"
  [ -n "$UR_PREFIX" ] && export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH:+$IGN_GAZEBO_RESOURCE_PATH:}${UR_PREFIX}/share"
fi

unset REPO_ROOT UR_PREFIX
