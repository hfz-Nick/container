cmake_minimum_required(VERSION 3.5)
project(navatics_can_interface)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(navatics_msgs REQUIRED)
find_package(serial REQUIRED)

# include directories
set(EIGEN3_INCLUDE_DIRS /usr/local/include/eigen3)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${navatics_msgs_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  library/can_interface_manager.cpp
)

# link library with rclcpp and other libraries in navatics_libs
link_directories(/usr/local/lib/navatics_libs)
ament_target_dependencies(${PROJECT_NAME} rclcpp navatics_msgs serial)

# add executable
add_executable(can_interface_node src/can_interface_node.cpp)
target_link_libraries(can_interface_node ${PROJECT_NAME})
ament_target_dependencies(can_interface_node rclcpp navatics_msgs serial)

# export and install targets
ament_export_include_directories(include)

install(TARGETS
   can_interface_node
   DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
