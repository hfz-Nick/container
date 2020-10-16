#!/bin/bash

if [ -z "$1" ]
  then
    CONTAINER_NAME="core"
  else
    CONTAINER_NAME="$1"
fi

ROS2_WS_DIRECTORY=$(cd `dirname $0` && pwd)

docker run --name ${CONTAINER_NAME} -it --rm \
       --net bridge --privileged \
       -v "${ROS2_WS_DIRECTORY}/core_ws/:/home/ros/core_ws/" \
       -v "${ROS2_WS_DIRECTORY}/core_sensors/:/home/ros/sensors/" \
       -v "${ROS2_WS_DIRECTORY}/core_params/:/home/ros/params/" \
       -v "${ROS2_WS_DIRECTORY}/core_tests/:/home/ros/tests/" \
       -v "/tmp/psu00/":"/tmp/psu00" \
       -v "/tmp/shutdown_cmd.txt":"/tmp/shutdown_cmd.txt" \
       -v "/dev/nvt_serial_mds":"/dev/nvt_serial_mds" \
       -v "/dev/nvt_serial_stm":"/dev/nvt_serial_stm" \
       -v "/dev/nvt_serial_gps":"/dev/nvt_serial_gps" \
       navatics_core:dev
