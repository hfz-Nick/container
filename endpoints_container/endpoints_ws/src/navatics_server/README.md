# `navatics_server`

This package contains code used to communicated between UUSV and ground control station based on MAVLink header-only library and UDP implementation.

## Dependencies

1. ROS2

2. MAVLink

## Building the Docker Image and ROS Packages

Follow unstructions in `uusv_endpoints_container` for more details

## Using the Package

The parameters needed to run this code can be found in `uusv_endpoints_container` repository

```
ros2 launch navatics_server navatics_endpoints.launch.py
```

## Work in Progress

1. Create header-only library installation for MAVLink so that the header files can be shared across multiple projects

2. Send quaternion and poisition messages
