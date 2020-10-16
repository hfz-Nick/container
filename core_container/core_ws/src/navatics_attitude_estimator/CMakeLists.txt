cmake_minimum_required(VERSION 3.5)
project(navatics_attitude_estimator)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include directories
set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
include_directories(
  /usr/local/include/navatics_libs
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  library/attitude_estimator.cpp
  library/attitude_predictor.cpp
  library/orientation.cpp
)

# link library with rclcpp and other libraries in navatics_libs
link_directories(/usr/local/lib/navatics_libs)
target_link_libraries(${PROJECT_NAME} navatics_i2c)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs)

# add executable
add_executable(mpu_unit_test test/mpu_unit_test.cpp)
target_link_libraries(mpu_unit_test ${PROJECT_NAME})
ament_target_dependencies(mpu_unit_test rclcpp std_msgs geometry_msgs)

add_executable(predictor_unit_test test/predictor_unit_test.cpp)
target_link_libraries(predictor_unit_test ${PROJECT_NAME})
ament_target_dependencies(predictor_unit_test rclcpp std_msgs geometry_msgs)

add_executable(attitude_estimator_node src/attitude_estimator_node.cpp)
target_link_libraries(attitude_estimator_node ${PROJECT_NAME})
ament_target_dependencies(attitude_estimator_node rclcpp std_msgs geometry_msgs)

# export and install targets
ament_export_include_directories(include)

install(TARGETS
  mpu_unit_test
  predictor_unit_test
  attitude_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
