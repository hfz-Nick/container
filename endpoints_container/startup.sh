#!/bin/bash

if [ -z "$1" ]
  then
    CONTAINER_NAME="endpoints"
  else
    CONTAINER_NAME="$1"
fi

ENDPOINTS_WS_DIRECTORY=$(cd `dirname $0` && pwd)

docker run --name ${CONTAINER_NAME} -it -d \
       --net host \
       -v "${ENDPOINTS_WS_DIRECTORY}/endpoints_params/:/home/ros/params/" \
       navatics_endpoints:deploy bash
