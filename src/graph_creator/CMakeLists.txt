cmake_minimum_required(VERSION 3.8)
project(graph_creator)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(graph_creator_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(DXFLIB REQUIRED dxflib)

# Build a library from all helper source files
add_library(graph_creator_lib 
    src/directed_graph.cpp
    src/dxf_graph_generator.cpp
    src/graph_curves.cpp
    src/graph_file_utils.cpp
    src/graph_generators.cpp
    src/graph_loader.cpp
    src/graph_saver.cpp
    src/graph_transformers.cpp
    src/graph_utils.cpp
    src/graph_visuals.cpp
)

target_include_directories(graph_creator_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${DXFLIB_INCLUDE_DIRS}
)

target_link_libraries(graph_creator_lib ${DXFLIB_LIBRARIES})

ament_target_dependencies(graph_creator_lib
    rclcpp
    visualization_msgs
    nav_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    ament_index_cpp
    graph_creator_msgs
    graph_msgs
)

# Build the main executable node
add_executable(graph_creator_node src/graph_creator.cpp)

ament_target_dependencies(graph_creator_node
    rclcpp
    graph_creator_msgs
    graph_msgs
)

target_link_libraries(graph_creator_node graph_creator_lib)

# Install all necessary files
install(TARGETS graph_creator_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS graph_creator_lib DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config/ layouts/ maps/ rviz/ DESTINATION share/${PROJECT_NAME}/)

ament_package()