cmake_minimum_required(VERSION 3.8)
project(brake_action)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# This command automatically finds all .cpp files in the src folder.
file(GLOB_RECURSE SOURCES "src/*.cpp")

# Build a single executable from all the discovered source files.
add_executable(brake_action_node ${SOURCES})

# Add the necessary include directories, including for libmodbus.
target_include_directories(brake_action_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

# Link against the libmodbus library.
target_link_libraries(brake_action_node
  ${MODBUS_LIBRARIES}
)

# Add dependencies on other ROS 2 packages.
ament_target_dependencies(brake_action_node
  rclcpp
  std_srvs
)

# Install the final executable.
install(TARGETS
  brake_action_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary files like launch and config.
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
