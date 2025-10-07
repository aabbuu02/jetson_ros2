cmake_minimum_required(VERSION 3.8)
project(tag_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# This builds the node from only the single, correct source file.
# This ignores any other old .cpp files that might exist.
add_executable(tag_monitor_node src/tag_monitor.cpp)

# Add the necessary include directories
target_include_directories(tag_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies on other ROS 2 packages
ament_target_dependencies(tag_monitor_node
  rclcpp
  std_msgs
  nav_msgs
  anscer_msgs
  tf2_geometry_msgs
)

# Install the final executable
install(TARGETS
  tag_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary files like launch and config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


