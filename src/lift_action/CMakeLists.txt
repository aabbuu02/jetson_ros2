cmake_minimum_required(VERSION 3.8)
project(lift_action)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find libmodbus system library
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Generate messages FIRST
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ErrorStatus.msg"
  "msg/HomeSensor.msg"
  "msg/HomingStatus.msg"
  "msg/LiftAction.msg"
  "msg/LiftFeedback.msg"
  "msg/UiInterface.msg"
  DEPENDENCIES std_msgs
)

# Get typesupport target - CRITICAL LINE
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Build ModbusCommunicator node
add_executable(modbus_communicator_node src/ModbusComunicator.cpp)

# Link dependencies
ament_target_dependencies(modbus_communicator_node rclcpp std_msgs)

# Link the generated messages and modbus
target_link_libraries(modbus_communicator_node
  "${cpp_typesupport_target}"
  ${MODBUS_LIBRARIES}
)

# Include directories
target_include_directories(modbus_communicator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MODBUS_INCLUDE_DIRS}
)

# Install executable
install(TARGETS
  modbus_communicator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and configs
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY
  include/
  DESTINATION include
)

ament_package()
