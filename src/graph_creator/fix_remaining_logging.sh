#!/bin/bash

# Fix remaining logging issues
sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Updated existing graph element "<<key_element<<" with new "<<value_element<<" dependency");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Updated existing graph element %s with new %s dependency", key_element.c_str(), value_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Added new graph element dependency between "<<key_element<<" and "<<value_element);/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Added new graph element dependency between %s and %s", key_element.c_str(), value_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Updated existing graph element "<<value_element<<" with new "<<key_element<<" dependency");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Updated existing graph element %s with new %s dependency", value_element.c_str(), key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Added new graph element dependency between "<<value_element<<" and "<<key_element);/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Added new graph element dependency between %s and %s", value_element.c_str(), key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Vertex "<<key_element<<" cannot be deleted for miscellaneous dependency until deleted from graph");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Vertex %s cannot be deleted for miscellaneous dependency until deleted from graph", key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge "<<key_element<<" cannot be deleted for miscellaneous dependency until deleted from graph");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Edge %s cannot be deleted for miscellaneous dependency until deleted from graph", key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Miscellaneous graph element key "<<key_element<<" not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Miscellaneous graph element key %s not found", key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Removing key "<<key_element<<" as no it has no dependency");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Removing key %s as no it has no dependency", key_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph element dependency between "<<key_element<<" and "<<value_element<<" deleted");/RCLCPP_INFO(rclcpp::get_logger("graph_creator"), "Graph element dependency between %s and %s deleted", key_element.c_str(), value_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Graph element dependency between "<<key_element<<" and "<<value_element<<" does not exist");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Graph element dependency between %s and %s does not exist", key_element.c_str(), value_element.c_str());/g' directed_graph.cpp

sed -i 's/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Miscellaneous "<<key_element<<" graph element dependencies not found");/RCLCPP_WARN(rclcpp::get_logger("graph_creator"), "Miscellaneous %s graph element dependencies not found", key_element.c_str());/g' directed_graph.cpp

echo "Remaining logging fixes applied to directed_graph.cpp"

