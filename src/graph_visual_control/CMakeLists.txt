cmake_minimum_required(VERSION 3.8)
project(graph_visual_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(graph_viz_ctrl_node src/graph_viz_ctrl.cpp)

ament_target_dependencies(graph_viz_ctrl_node
  rclcpp
  visualization_msgs
  graph_msgs
  tf2
)

target_include_directories(graph_viz_ctrl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  graph_viz_ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
