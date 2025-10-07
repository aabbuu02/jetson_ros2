cmake_minimum_required(VERSION 3.8)
project(emergency_handler)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(anscer_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(emergency_handler_node src/emergency_handler.cpp)

ament_target_dependencies(emergency_handler_node
  rclcpp
  anscer_msgs
  geometry_msgs
)

install(TARGETS
  emergency_handler_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
