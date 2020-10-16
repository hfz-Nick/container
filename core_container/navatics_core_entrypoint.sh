#!/bin/bash
set -e

# setup ros2 environment
source "/home/ros/libraries/ros2_libs/install/setup.bash"
exec "$@"
