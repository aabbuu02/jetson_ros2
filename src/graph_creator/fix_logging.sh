#!/bin/bash

# Fix all stream-style logging in directed_graph.cpp
sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Vertex "<<vertex_name<<" to edge dependencies not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Vertex %s to edge dependencies not found", vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Source vertex "<<src_vertex_name<<" and destination vertex "<<dst_vertex_name<<" not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Source vertex %s and destination vertex %s not found", src_vertex_name.c_str(), dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Source vertex "<<src_vertex_name<<" not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Source vertex %s not found", src_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Destination vertex "<<dst_vertex_name<<" not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Destination vertex %s not found", dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" added");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Edge between vertex %s and %s added", src_vertex_name.c_str(), dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" already exists");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge between vertex %s and %s already exists", src_vertex_name.c_str(), dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Edge "<<edge_name<<" could not be found. Initial edge creation probably failed!");/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Edge %s could not be found. Initial edge creation probably failed!", edge_name.c_str());/g' directed_graph.cpp

# Fix remaining complex logging statements
sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" deleted");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Edge between vertex %s and %s deleted", src_vertex_name.c_str(), dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge between vertex "<<src_vertex_name<<" and "<<dst_vertex_name<<" does not exist");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge between vertex %s and %s does not exist", src_vertex_name.c_str(), dst_vertex_name.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge "<<edge_name<<" to vertex dependencies not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge %s to vertex dependencies not found", edge_name.c_str());/g' directed_graph.cpp

# Fix debug statements with stringstream
sed -i 's/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Graph\\n"<<ss_gph.str());/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Graph\\n%s", ss_gph.str().c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Edge list\\n"<<ss_el.str());/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Edge list\\n%s", ss_el.str().c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Vertex to edges dependency list\\n"<<ss_v2e_dep.str());/RCLCPP_DEBUG(rclcpp::get_logger("graph_creator"), "Vertex to edges dependency list\\n%s", ss_v2e_dep.str().c_str());/g' directed_graph.cpp

echo "Logging fixes applied to directed_graph.cpp"

