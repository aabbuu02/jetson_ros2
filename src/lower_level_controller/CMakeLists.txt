cmake_minimum_required(VERSION 2.8.3)
project(lower_level_controller)

add_compile_options(-std=c++11)
add_definitions(-Dlinux)


find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  roscpp
  rospy
  std_msgs
  roslib
  tf
  dynamic_reconfigure
  anscer_msgs
  actionlib 
  roslib
  lift_action
  
)

generate_dynamic_reconfigure_options(cfg/lowerLevelController.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    
    geometry_msgs
    roscpp
    rospy
    std_msgs
    actionlib
    lift_action
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/lower_level_controller
)
set(CMAKE_CXX_STANDARD_LIBRARIES)

add_executable(lower_level_controller
  src/LowerLevelInterface.cpp
  src/LowerLevelComputations.cpp
  src/RosClass.cpp

  src/ModbusController.cpp
  src/ModbusDevice.cpp

#  src/ModbusRoboteqController.cpp
#src/RoboteqDevice.cpp

#  src/ODriveController.cpp
#  src/ODriveDevice.cpp
#  src/CommunicationSerial.cpp

#  src/RainbowController.cpp
#  src/RainbowDevice.cpp
)

target_link_libraries(lower_level_controller
  ${catkin_LIBRARIES}
  modbus 
)

add_dependencies(lower_level_controller ${PROJECT_NAME}_gencfg)

## Mark other files for installation (e.g. launch and bag files, etc.)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include launch config data cfg
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
 install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
