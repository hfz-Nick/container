# `navatics_position_estimator`

This package contains code that estimate position of an ROV using a pressure sensor (MS5837). Further development will allow for position update from another device, such as a USBL system.

## Dependencies

1. [`navatics_i2c`](http://192.168.69.40/rov_dev_libs/cpp/navatics_i2c)

2. Eigen

## Building the Docker Image and ROS packages

Follow instructions in `uusv_core_container` for more details


## Running the ROS2 node

The parameters needed to run this code, together will the format for calibration files can be found in `uusv_core_container` repository.

```
ros2 run navatics_position_estimator position_estimator_node --ros-args --params-file ~/params/position_estimator_params.yaml
```

