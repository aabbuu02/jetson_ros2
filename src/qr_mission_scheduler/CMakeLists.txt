cmake_minimum_required(VERSION 3.8)
project(qr_mission_scheduler)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Generate messages, services, and actions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalMsg.msg"
  "srv/GetGoal.srv"
  "action/MissionScheduler.action"  # <-- CORRECTED FILENAME
)

# Build the server executable
add_executable(mission_scheduler_server src/mission_scheduler.cpp)
ament_target_dependencies(mission_scheduler_server rclcpp rclcpp_action)
target_link_libraries(mission_scheduler_server ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(mission_scheduler_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build the client executable
add_executable(qr_mission_client src/qr_mission_client.cpp)
ament_target_dependencies(qr_mission_client rclcpp rclcpp_action)
target_link_libraries(qr_mission_client ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(qr_mission_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install all necessary files
install(TARGETS
  mission_scheduler_server
  qr_mission_client
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/)

ament_package()
