cmake_minimum_required(VERSION 3.8)
project(anscer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ACRControl.msg"
  "msg/BatteryState.msg"
  "msg/Debug.msg"
  "msg/DockInfo.msg"
  "msg/ChargeInfo.msg"
  "msg/ControllerMode.msg"
  "msg/Feedback.msg"
  "msg/FieldStatus.msg"
  "msg/Indication.msg"
  "msg/KeyValue.msg"
  "msg/KeyValues.msg"
  "msg/MissionData.msg"
  "msg/MissionDB.msg"
  "msg/MissionInfo.msg"
  "msg/MotorDiagnostics.msg"
  "msg/MotorDiagnosticsArray.msg"
  "msg/MotorFaultFlags.msg"
  "msg/MotorStatusFlags.msg"
  "msg/PGVPose.msg"
  "msg/SafetyField.msg"
  "msg/SafetyFields.msg"
  "msg/Task.msg"
  "msg/ZonePoints.msg"
  "msg/ZoneInformation.msg"
  "srv/BatteryManagement.srv"
  "srv/ChargingOverride.srv"
  "srv/ClearZones.srv"
  "srv/HomeLocation.srv"
  "srv/MissionAnalytics.srv"
  "srv/NearestNode.srv"
  "srv/PausingFeedback.srv"
  "srv/RelayState.srv"
  "srv/Safety.srv"
  "srv/SafetySwitch.srv"
  "srv/SetZone.srv"
  "srv/ShelfReader.srv"
  "srv/TripAnalytics.srv"
  "srv/UserFeedback.srv"
  "srv/SafetyZone.srv"
  "action/ACRModule.action"
  "action/Charge.action"
  "action/Error.action"
  "action/Lift.action"
  "action/Move.action"
  "action/Mission.action"
  "action/Process.action"
  "action/Wait.action"
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()