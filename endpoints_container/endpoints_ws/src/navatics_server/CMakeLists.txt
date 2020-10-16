cmake_minimum_required(VERSION 3.10)
project (navatics_server)

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(navatics_msgs REQUIRED)

include_directories(
  include
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${navatics_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  library/endpoints.cpp
  library/mavlink_msg_handler.cpp
  library/mavlink_server.cpp
  library/mavlink_state_pub.cpp
)

add_executable(mavlink_server_test test/mavlink_server_test.cpp)
target_link_libraries(mavlink_server_test ${PROJECT_NAME})
ament_target_dependencies(mavlink_server_test rclcpp geometry_msgs nav_msgs std_msgs navatics_msgs)

add_executable(endpoints_node src/endpoints_node.cpp)
target_link_libraries(endpoints_node ${PROJECT_NAME})
ament_target_dependencies(endpoints_node rclcpp geometry_msgs nav_msgs std_msgs navatics_msgs)

# export and install targets
ament_export_include_directories(include)

install(TARGETS
  mavlink_server_test 
  endpoints_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
