cmake_minimum_required(VERSION 2.8.3)
project(acr_robot_controller)

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
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    sensor_msgs 
    geometry_msgs
    roscpp
    rospy
    std_msgs
    actionlib
)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/acr_robot_controller
)
#set(CMAKE_CXX_STANDARD_LIBRARIES -ludev)

add_executable(acr_robot_controller
  src/LowerLevelInterface.cpp
  src/LowerLevelComputations.cpp
  src/RosClass.cpp


  src/RoboteqController.cpp
  src/RoboteqDevice.cpp

)

target_link_libraries(acr_robot_controller
  ${catkin_LIBRARIES}
  modbus
)



## Mark other files for installation (e.g. launch and bag files, etc.)




install(TARGETS acr_robot_controller
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
