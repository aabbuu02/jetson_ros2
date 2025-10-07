#ifndef PGV_READER_H
#define PGV_READER_H

#include <iostream>
#include <ros/ros.h>
#include <cstring>
#include <stdio.h>
#include <ros/package.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <anscer_msgs/PGVPose.h>

#include <unordered_map>
#include "angles/angles.h"
// #include <sqlite3.h>
#include <libserial/SerialPort.h>

// #include <SerialStream.h>

struct BarcodeInfo
{
    double barcodeToMapX;
    double barcodeToMapY;
};

class PGVReader
{
public:
    PGVReader();
    ~PGVReader();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nhp;

    ros::Publisher localPosePub;
    ros::Publisher globalPosePub;
    ros::Publisher nodeMarkerPub;

    /*Member functions*/
    void dataReadCallback(const ros::TimerEvent &event);
    void initializeParameters();
    void initializeConnection();
    void sendTypeToScan();
    void sendRequestToScan();
    void mapVisualization();
    // void readBarCodeInfoFromDB();
    void readBarCodeInfoFromCSV();
    std::string convertToString(char *a, int size);

    /*Member variables*/
    ros::Timer dataTimer;
    int m_baudRate;
    int m_tagId;

    double m_angleOffset;
    double m_scaleX, m_scaleY;

    double m_loopRate = 50.0;
    bool m_checkSumState = false;
    bool m_scanDetected = false;
    double m_pgvLocalX, m_pgvLocalY, m_pgvLocalAngle;
    double m_pgvGlobalX, m_pgvGlobalY, m_pgvGlobalAngle;

    std::string m_portName, m_packagePath, m_csvFileName;
    std::string m_barcodeLocalTopic, m_barcodeGlobalTopic;
    std::unordered_map<int, BarcodeInfo> m_barCodeMap;

    boost::shared_ptr<LibSerial::SerialPort> p_serialPort;

    visualization_msgs::MarkerArray m_markerArray;
};
#endif
