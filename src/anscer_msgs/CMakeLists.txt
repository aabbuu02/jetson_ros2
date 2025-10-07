cmake_minimum_required(VERSION 2.8.3)
project(anscer_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  geometry_msgs
)



## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ACRControl.msg
  BatteryState.msg
  Debug.msg
  DockInfo.msg  
  ChargeInfo.msg  
  ControllerMode.msg  
  Feedback.msg
  FieldStatus.msg
  Indication.msg
  KeyValue.msg
  KeyValues.msg
  MissionData.msg
  MissionDB.msg
  MissionInfo.msg
  MotorDiagnostics.msg
  MotorDiagnosticsArray.msg
  MotorFaultFlags.msg
  MotorStatusFlags.msg
  PGVPose.msg
  SafetyField.msg
  SafetyFields.msg
  Task.msg	
  ZonePoints.msg
  ZoneInformation.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   BatteryManagement.srv
   ChargingOverride.srv
   ClearZones.srv
   HomeLocation.srv
   MissionAnalytics.srv
   NearestNode.srv
   PausingFeedback.srv
   RelayState.srv
   Safety.srv
   SafetySwitch.srv
   SetZone.srv
   ShelfReader.srv
   TripAnalytics.srv
   UserFeedback.srv
   SafetyZone.srv
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   ACRModule.action
   Charge.action
   Error.action
   Lift.action
   Move.action
   Mission.action
   Process.action
   Wait.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agv_msgs
  CATKIN_DEPENDS message_runtime std_msgs actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
