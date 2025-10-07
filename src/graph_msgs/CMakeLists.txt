cmake_minimum_required(VERSION 3.8)
project(graph_msgs)

# Find necessary build tools and dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# This is the command that generates the message and service files
# Corrected "Vertex.msg" to "Node.msg"
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Edge.msg"
  "msg/Graph.msg"
  "msg/GraphMetaData.msg"
  "msg/Property.msg"
  "msg/Node.msg"
  "srv/GetGraph.srv"
  "srv/GetGraphPlan.srv"
  "srv/LoadGraph.srv"
  "srv/SaveGraph.srv"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

# This is the crucial section that was missing.
# It ensures that downstream packages know about the runtime dependencies.
ament_export_dependencies(rosidl_default_runtime)

# This is the other crucial missing section.
# It installs the generated headers and other necessary files so that
# other packages can find and use them.
install(
  TARGETS ${rosidl_get_targets_for_interface_files}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "msg" "srv"
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


