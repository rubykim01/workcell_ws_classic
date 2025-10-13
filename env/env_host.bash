#!/usr/bin/env bash
# Host-side environment setup

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=127
export LC_ALL=C
export LANG=C

# Source link_attacher if installed
if [ -f /opt/link_attacher/install/setup.bash ]; then
  source /opt/link_attacher/install/setup.bash
fi

echo "[host] ROS env loaded:"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
