#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Source workspace if exists
if [ -f /ros_ws/install/setup.bash ]; then
    source /ros_ws/install/setup.bash
fi

exec "$@"
