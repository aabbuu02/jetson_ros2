cmake_minimum_required(VERSION 3.8)
project(reader_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Build the executables
add_executable(pgv_reader_node
  src/pgv_reader_ros2.cpp
)

add_executable(shelf_reader_node
  src/shelf_reader_ros2.cpp
)

ament_target_dependencies(pgv_reader_node
  rclcpp
  geometry_msgs
  std_msgs
  anscer_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(shelf_reader_node
  rclcpp
  geometry_msgs
  std_msgs
  anscer_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
)

# Install the executables
install(TARGETS pgv_reader_node shelf_reader_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()