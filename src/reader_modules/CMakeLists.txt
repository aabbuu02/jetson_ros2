cmake_minimum_required(VERSION 2.8.3)
project(reader_modules)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  roscpp
  rospy
  std_msgs
  anscer_msgs
  roslib
)










catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    
    geometry_msgs
    roscpp
    rospy
    std_msgs
    anscer_msgs

  
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/reader_modules
)

add_executable(shelf_reader
  src/shelf_reader.cpp
)

link_directories(/usr/local/lib)

target_link_libraries(shelf_reader
  ${catkin_LIBRARIES} 
)

add_executable(pgv_reader
  src/pgv_reader.cpp
)

target_link_libraries(pgv_reader
  ${catkin_LIBRARIES} serial sqlite3
)


install(TARGETS shelf_reader pgv_reader
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY include launch data
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )
