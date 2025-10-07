/**
 * @file pgv_reader.h
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#ifndef PGV_READER_H
#define PGV_READER_H

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <stdio.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <anscer_msgs/msg/pgv_pose.hpp>
#include <unordered_map>
#include <cmath>
#include <libserial/SerialPort.h>

struct BarcodeInfo
{
    double barcodeToMapX;
    double barcodeToMapY;
};

class PGVReader : public rclcpp::Node
{
public:
    PGVReader();
    ~PGVReader();

private:
    // Publishers
    rclcpp::Publisher<anscer_msgs::msg::PGVPose>::SharedPtr localPosePub;
    rclcpp::Publisher<anscer_msgs::msg::PGVPose>::SharedPtr globalPosePub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nodeMarkerPub;
    
    // Timer
    rclcpp::TimerBase::SharedPtr dataTimer;

    // Member functions
    void dataReadCallback();
    void initializeParameters();
    void initializeConnection();
    void sendTypeToScan();
    void sendRequestToScan();
    void mapVisualization();
    void readBarCodeInfoFromCSV();
    std::string convertToString(char *a, int size);
    double normalize_angle(double angle);

    // Member variables
    int m_baudRate;
    int m_tagId;
    double m_angleOffset;
    double m_loopRate = 50.0;
    bool m_checkSumState = false;
    bool m_scanDetected = false;
    double m_pgvLocalX, m_pgvLocalY, m_pgvLocalAngle;
    double m_pgvGlobalX, m_pgvGlobalY, m_pgvGlobalAngle;
    std::string m_portName, m_packagePath, m_csvFileName;
    std::string m_barcodeLocalTopic, m_barcodeGlobalTopic;
    std::unordered_map<int, BarcodeInfo> m_barCodeMap;
    std::shared_ptr<LibSerial::SerialPort> p_serialPort;
    visualization_msgs::msg::MarkerArray m_markerArray;
};

#endif
