#!/bin/bash

# Fix logging in dxf_graph_generator.cpp
sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "DXF file "<<dxf_file_path<<" loaded successfully");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "DXF file %s loaded successfully", dxf_file_path.c_str());/g' dxf_graph_generator.cpp

sed -i 's/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "DXF file "<<dxf_file_path<<" could not be loaded");/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "DXF file %s could not be loaded", dxf_file_path.c_str());/g' dxf_graph_generator.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "DXF file "<<dxf_file_path<<" is empty");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "DXF file %s is empty", dxf_file_path.c_str());/g' dxf_graph_generator.cpp

# Fix logging in graph_file_utils.cpp
sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" saved successfully");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file %s saved successfully", file_path.c_str());/g' graph_file_utils.cpp

sed -i 's/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" could not be saved");/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file %s could not be saved", file_path.c_str());/g' graph_file_utils.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" loaded successfully");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file %s loaded successfully", file_path.c_str());/g' graph_file_utils.cpp

sed -i 's/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" could not be loaded");/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file %s could not be loaded", file_path.c_str());/g' graph_file_utils.cpp

# Fix logging in graph_loader.cpp
sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" loaded successfully");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph file %s loaded successfully", file_path.c_str());/g' graph_loader.cpp

sed -i 's/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" could not be loaded");/RCLCPP_ERROR(rclcpp::get_logger("graph_creator"), "Graph file %s could not be loaded", file_path.c_str());/g' graph_loader.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Graph file "<<file_path<<" is empty");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Graph file %s is empty", file_path.c_str());/g' graph_loader.cpp

echo "All logging fixes applied to all source files"

