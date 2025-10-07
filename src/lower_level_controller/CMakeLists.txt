cmake_minimum_required(VERSION 3.8)
project(lower_level_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Build ONE single executable from an EXPLICIT list of required source files
add_executable(lower_level_controller_node
  src/LowerLevelInterface.cpp
  src/RosClass.cpp
  src/LowerLevelComputations.cpp
  src/ModbusController.cpp
  src/ModbusDevice.cpp
)

ament_target_dependencies(lower_level_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(lower_level_controller_node
  ${MODBUS_LIBRARIES}
)
target_include_directories(lower_level_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

install(TARGETS
  lower_level_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
