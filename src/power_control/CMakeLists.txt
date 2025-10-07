cmake_minimum_required(VERSION 3.8)
project(power_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

# This automatically finds all .cpp files in the src folder
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Build a single executable from all the discovered source files
add_executable(power_control_node ${SOURCES})

# Add the necessary include directories
target_include_directories(power_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Add dependencies on other ROS 2 packages
ament_target_dependencies(power_control_node
  rclcpp
  std_srvs
)

# Install the final executable
install(TARGETS
  power_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary files like launch and config
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

