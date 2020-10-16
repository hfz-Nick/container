cmake_minimum_required(VERSION 3.5)
project(navatics_position_estimator)

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
  library/position_estimator.cpp
  library/depth_predictor.cpp
  library/position.cpp
)

# link library with rclcpp and other libraries in navatics_libs
link_directories(/usr/local/lib/navatics_libs)
target_link_libraries(${PROJECT_NAME} navatics_i2c)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs geometry_msgs)

# add executable
add_executable(psu_unit_test test/psu_unit_test.cpp)
target_link_libraries(psu_unit_test ${PROJECT_NAME})
ament_target_dependencies(psu_unit_test rclcpp std_msgs geometry_msgs)

add_executable(position_estimator_node src/position_estimator_node.cpp)
target_link_libraries(position_estimator_node ${PROJECT_NAME})
ament_target_dependencies(position_estimator_node rclcpp std_msgs geometry_msgs)

# export and install targets
ament_export_include_directories(include)

install(TARGETS
  psu_unit_test
  position_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
