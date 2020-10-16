cmake_minimum_required(VERSION 3.5)
project(navatics_robot_controller)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navatics_msgs REQUIRED)
find_package(serial REQUIRED)

# include directories
set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${navatics_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  library/controller.cpp
  library/position.cpp
  library/orientation.cpp
  library/refstate.cpp
  library/thrusters.cpp
)

# link library with rclcpp and other libraries in navatics_libs
link_directories(/usr/local/lib/navatics_libs)
ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs nav_msgs navatics_msgs serial)

# add executable
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node ${PROJECT_NAME})
ament_target_dependencies(controller_node rclcpp geometry_msgs nav_msgs navatics_msgs serial)

# export and install targets
ament_export_include_directories(include)

install(TARGETS
   controller_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

