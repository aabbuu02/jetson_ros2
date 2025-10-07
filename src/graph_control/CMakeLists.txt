cmake_minimum_required(VERSION 3.8)
project(graph_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)

add_executable(graph_control_loader_node src/graph_control_plugin_loader.cpp)

ament_target_dependencies(graph_control_loader_node
  rclcpp
  pluginlib
  geometry_msgs
  graph_msgs
)

target_include_directories(graph_control_loader_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  graph_control_loader_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
