#!/usr/bin/env bash
set -e

if [[ -f /opt/ros2_ws/install/setup.bash ]]; then
    source /opt/ros2_ws/install/setup.bash
fi

exec "$@"
