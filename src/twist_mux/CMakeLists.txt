cmake_minimum_required(VERSION 3.8)
project(twist_mux)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(twist_mux_node src/twist_mux.cpp)

target_include_directories(twist_mux_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(twist_mux_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  twist_mux_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

