#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f /swarm_ws/install/setup.bash ]; then
    source /swarm_ws/install/setup.bash
fi

# Start virtual display
Xvfb :99 -screen 0 1920x1080x24 &
export DISPLAY=:99

# Start window manager
openbox &

# Start VNC server (no password, port 5900)
x11vnc -display :99 -forever -nopw -quiet &

exec "$@"