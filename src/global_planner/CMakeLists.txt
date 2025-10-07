cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the executable
add_executable(path_graph_planner_node 
  src/path_graph_planner_node.cpp 
  src/dijkstra_graph_planner.cpp 
  src/global_path_graph.cpp 
  src/graph_curves.cpp
)

ament_target_dependencies(path_graph_planner_node
  rclcpp
  nav_msgs
  graph_msgs
  geometry_msgs
  tf2_geometry_msgs
)

# Install the executable
install(TARGETS path_graph_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
