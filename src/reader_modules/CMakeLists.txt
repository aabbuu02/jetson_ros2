cmake_minimum_required(VERSION 3.8)
project(reader_modules)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ALL ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find system libraries
find_package(PkgConfig REQUIRED)
pkg_check_modules(SERIAL REQUIRED libserial)
find_package(SQLite3 REQUIRED)

# Include directories
include_directories(
  include
  ${SERIAL_INCLUDE_DIRS}
  ${SQLite3_INCLUDE_DIRS}
)

# Build shelf_reader executable
add_executable(shelf_reader src/shelf_reader.cpp)
ament_target_dependencies(shelf_reader
  rclcpp
  geometry_msgs
  std_msgs
  anscer_msgs
)

# Build pgv_reader executable
add_executable(pgv_reader src/pgv_reader.cpp)
ament_target_dependencies(pgv_reader
  rclcpp
  geometry_msgs
  std_msgs
  anscer_msgs
  visualization_msgs
  ament_index_cpp
  tf2
  tf2_geometry_msgs
)
target_link_libraries(pgv_reader
  ${SERIAL_LIBRARIES}
  ${SQLite3_LIBRARIES}
)

# Install executables
install(TARGETS
  shelf_reader
  pgv_reader
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  include/
  launch
  data
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
