cmake_minimum_required(VERSION 3.8)
project(emergency_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the executable
add_executable(emergency_handler_node
  src/emergency_handler_ros2.cpp
)

ament_target_dependencies(emergency_handler_node
  rclcpp
  geometry_msgs
  std_msgs
  anscer_msgs
)

# Install the executable
install(TARGETS emergency_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()