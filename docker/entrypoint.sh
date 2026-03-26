#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f /swarm_ws/install/setup.bash ]; then
    source /swarm_ws/install/setup.bash
fi

exec "$@"
