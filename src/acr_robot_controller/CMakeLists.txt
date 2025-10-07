cmake_minimum_required(VERSION 3.8)
project(acr_robot_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build a single executable from our one, self-contained source file.
add_executable(acr_robot_controller_node src/acr_robot_controller_node.cpp)

# Add dependencies on other ROS 2 packages
ament_target_dependencies(acr_robot_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  anscer_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install the final executable
install(TARGETS
  acr_robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other necessary files
install(
  DIRECTORY launch config udev
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


