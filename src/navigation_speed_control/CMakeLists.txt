cmake_minimum_required(VERSION 2.8.3)
project(navigation_speed_control)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  roscpp
  rospy
  std_msgs
  anscer_msgs
  roslib
  geometry_msgs
  dynamic_reconfigure

  

)










catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    
    roscpp
    rospy
    std_msgs
    anscer_msgs
    geometry_msgs
    dynamic_reconfigure
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/navigation_speed_control
)


add_executable(navigation_speed_control
  src/navigation_speed_control.cpp
)

target_link_libraries(navigation_speed_control
  ${catkin_LIBRARIES} 
)


## Mark executables for installation

install(TARGETS navigation_speed_control 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include launch config
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
