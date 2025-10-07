cmake_minimum_required(VERSION 3.8)
project(graph_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2_vendor REQUIRED)

# Add executable
add_executable(graph_server_node src/graph_server_node.cpp)
ament_target_dependencies(graph_server_node
  rclcpp
  graph_msgs
  ament_index_cpp
  tinyxml2_vendor
)

# Try linking to tinyxml2::tinyxml2; fallback to system tinyxml2 if not found
find_package(PkgConfig)
if(PkgConfig_FOUND)
  pkg_check_modules(TINYXML2 tinyxml2)
  if(TINYXML2_FOUND)
    target_link_libraries(graph_server_node ${TINYXML2_LIBRARIES})
    target_include_directories(graph_server_node PUBLIC ${TINYXML2_INCLUDE_DIRS})
  else()
    # Try tinyxml2::tinyxml2
    target_link_libraries(graph_server_node tinyxml2::tinyxml2)
  endif()
else()
  # Fallback to tinyxml2::tinyxml2
  target_link_libraries(graph_server_node tinyxml2::tinyxml2)
endif()

# Install executable
install(TARGETS graph_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install graphs directory
install(DIRECTORY graphs
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

