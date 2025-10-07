    cmake_minimum_required(VERSION 3.8)
    project(graph_creator_msgs)

    find_package(ament_cmake REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(graph_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/EdgeCreationType.msg"
      "msg/EdgeDirectionCreationType.msg"
      "msg/GraphCreatorMode.msg"
      "msg/GraphCreatorState.msg"
      "msg/GraphGeneratorConfig.msg"
      "msg/SelectedGraphElements.msg"
      "msg/SnapPoseConfig.msg"
      "srv/GenerateGraph.srv"
      "srv/GetGraphCreatorState.srv"
      "srv/SetEdgeCreationType.srv"
      "srv/SetEdgeDirectionCreationType.srv"
      "srv/SetGraphCreatorMode.srv"
      "srv/SetSnapPose.srv"
      DEPENDENCIES std_msgs geometry_msgs graph_msgs
    )

    ament_package()
