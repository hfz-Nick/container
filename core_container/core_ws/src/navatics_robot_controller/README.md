# `navatics_attitude_estimator`

This package contains code that estimate attitude of an ROV using 2 MPUs.

## Dependencies

1. Eigen

2. `navatics_msgs`

## Building the Docker Image and ROS packages

Follow instructions in `uusv_core_container` for more details


## Running the ROS2 node

The parameters needed to run this code can be found in `uusv_core_container` repository.

```
ros2 run navatics_robot_controller controller_node --ros-args --params-file ~/params/controller_params.yaml --params-file ~/params/thrusters_params.yaml
```

