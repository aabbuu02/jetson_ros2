/**
 * @file pgv_reader_ros2.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief ROS2 PGV reader matching ROS1 functionality
 */

#include "rclcpp/rclcpp.hpp"
#include "anscer_msgs/msg/pgv_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <memory>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

class PGVReader : public rclcpp::Node
{
public:
  PGVReader()
  : Node("pgv_reader_node")
  {
    // Declare parameters
    this->declare_parameter("barcode_local_topic", "/barcode/local");
    this->declare_parameter("barcode_global_topic", "/barcode/global");
    this->declare_parameter("loop_rate", 10.0);
    this->declare_parameter("csv_file_path", "/opt/zeus_ros2_ws/src/reader_modules/data/barcode_info.csv");

    // Get parameters
    m_barcode_local_topic = this->get_parameter("barcode_local_topic").as_string();
    m_barcode_global_topic = this->get_parameter("barcode_global_topic").as_string();
    m_loop_rate = this->get_parameter("loop_rate").as_double();
    m_csv_file_path = this->get_parameter("csv_file_path").as_string();

    // Create publishers
    m_local_pose_pub = this->create_publisher<anscer_msgs::msg::PGVPose>(m_barcode_local_topic, 10);
    m_global_pose_pub = this->create_publisher<anscer_msgs::msg::PGVPose>(m_barcode_global_topic, 10);
    m_node_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/barcode_map", 10);

    // Initialize TF2
    m_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);

    // Read barcode info from CSV
    readBarCodeInfoFromCSV();

    // Create timer for data reading
    m_data_timer = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / m_loop_rate)),
      std::bind(&PGVReader::dataReadCallback, this));

    RCLCPP_INFO(this->get_logger(), "PGV Reader node has been started.");
  }

private:
  void readBarCodeInfoFromCSV()
  {
    std::ifstream file(m_csv_file_path);
    if (!file.is_open()) {
      RCLCPP_WARN(this->get_logger(), "Could not open CSV file: %s", m_csv_file_path.c_str());
      return;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string cell;
      std::vector<std::string> row;
      
      while (std::getline(ss, cell, ',')) {
        row.push_back(cell);
      }
      
      if (row.size() >= 4) {
        BarcodeInfo info;
        info.id = std::stoi(row[0]);
        info.x = std::stod(row[1]);
        info.y = std::stod(row[2]);
        info.theta = std::stod(row[3]);
        
        m_barcode_map[info.id] = info;
      }
    }
    
    file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu barcode entries from CSV", m_barcode_map.size());
  }

  void dataReadCallback()
  {
    // Simulate reading PGV data (replace with actual PGV reading logic)
    // For now, we'll publish a dummy pose
    anscer_msgs::msg::PGVPose local_pose;
    local_pose.header.stamp = this->get_clock()->now();
    local_pose.header.frame_id = "base_link";
    local_pose.pose.position.x = 0.0;
    local_pose.pose.position.y = 0.0;
    local_pose.pose.position.z = 0.0;
    local_pose.pose.orientation.x = 0.0;
    local_pose.pose.orientation.y = 0.0;
    local_pose.pose.orientation.z = 0.0;
    local_pose.pose.orientation.w = 1.0;
    local_pose.barcode_id = 1; // Dummy barcode ID

    // Publish local pose
    m_local_pose_pub->publish(local_pose);

    // Transform to global frame if possible
    try {
      auto transform = m_tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
      
      anscer_msgs::msg::PGVPose global_pose = local_pose;
      global_pose.header.frame_id = "map";
      
      // Transform pose to global frame
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = local_pose.header;
      pose_stamped.pose = local_pose.pose;
      
      auto transformed_pose = tf2::doTransform(pose_stamped, transform);
      global_pose.pose = transformed_pose.pose;
      
      m_global_pose_pub->publish(global_pose);
    } catch (tf2::TransformException &ex) {
      RCLCPP_DEBUG(this->get_logger(), "Could not transform pose: %s", ex.what());
    }
  }

  struct BarcodeInfo {
    int id;
    double x, y, theta;
  };

  // Publishers
  rclcpp::Publisher<anscer_msgs::msg::PGVPose>::SharedPtr m_local_pose_pub;
  rclcpp::Publisher<anscer_msgs::msg::PGVPose>::SharedPtr m_global_pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_node_marker_pub;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

  // Timer
  rclcpp::TimerBase::SharedPtr m_data_timer;

  // Member variables
  std::string m_barcode_local_topic;
  std::string m_barcode_global_topic;
  double m_loop_rate;
  std::string m_csv_file_path;
  std::map<int, BarcodeInfo> m_barcode_map;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PGVReader>());
  rclcpp::shutdown();
  return 0;
}