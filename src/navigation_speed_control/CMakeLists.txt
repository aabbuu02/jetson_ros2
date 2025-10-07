cmake_minimum_required(VERSION 3.8)
project(navigation_speed_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(navigation_speed_control_node src/navigation_speed_control.cpp)

ament_target_dependencies(navigation_speed_control_node
  rclcpp
  geometry_msgs
  nav_msgs
)

target_include_directories(navigation_speed_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  navigation_speed_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
