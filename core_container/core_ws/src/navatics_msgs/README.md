# `navatics_msgs`

This package contains custom messages that is used in the ROS2 code deployment

## Dependencies

1. ROS2 

## Building the Docker Image and ROS packages

Follow instructions in `uusv_core_container` for more details

## Using the Custom Message

1. In the `CMakeLists.txt` of the intended package, add the following:
  ```
  find_package(navatics_msgs REQUIRED)
  ```
  and link it to the target library via `target_link_libraries`

2. In the `package.xml` add:
  ```
  <build_depend>navatics_msgs</build_depend>
  <exec_depend>navatics_msgs</exec_depend>
  ```
