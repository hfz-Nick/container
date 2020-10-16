#!/bin/bash
set -e

# setup ros2 environment
source "/home/ros/endpoints_ws/install/setup.bash"
exec "$@"

