cmake_minimum_required(VERSION 2.8.3)
project(emergency_handler)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  roscpp
  rospy
  std_msgs
  roslib
  anscer_msgs
 

  

)










catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    
    geometry_msgs
    roscpp
    rospy
    std_msgs

)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/
)


add_executable(emergency_handler
  src/emergency_handler.cpp

)

target_link_libraries(emergency_handler
  ${catkin_LIBRARIES}
)


## Mark executables for installation

install(TARGETS emergency_handler 
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
