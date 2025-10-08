cmake_minimum_required(VERSION 3.7.2)
project(wms_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)



## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   WMSData.msg
   Feedback.msg
   WMSTasks.msg

 )



catkin_python_setup()

catkin_install_python(
  PROGRAMS scripts/sql1.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )
