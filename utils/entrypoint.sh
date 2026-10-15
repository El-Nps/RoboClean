#!/usr/bin/env bash
set -e

# Source ROS 2 base
source /opt/ros/jazzy/setup.bash

# Check for workspace
if [ -f "/opt/robot_ws/install/setup.bash" ]; then
    source "/opt/robot_ws/install/setup.bash"
    # Set default ROS_DOMAIN_ID if not externally provided
    if [ -z "$ROS_DOMAIN_ID" ]; then
        export ROS_DOMAIN_ID=30
    fi
fi

exec "$@"
