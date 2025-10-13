#!/usr/bin/env bash
# Container-side environment setup

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=127
export LC_ALL=C
export LANG=C

# Source ROS 2 setup
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Source workspace if built
if [ -f /root/workcell_ws_classic/install/setup.bash ]; then
  source /root/workcell_ws_classic/install/setup.bash
fi

echo "[container] ROS env loaded:"
echo "  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
