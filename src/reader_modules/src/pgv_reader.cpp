#include "pgv_reader.h"
#include <ros/console.h>

/**
 * @brief  Constructor for the PGVReader
 */
PGVReader::PGVReader() : nhp("~")
{
    initializeParameters();
    initializeConnection();
    readBarCodeInfoFromCSV();
    localPosePub      = nh.advertise<anscer_msgs::PGVPose>(m_barcodeLocalTopic.c_str(), 10);
    globalPosePub     = nh.advertise<anscer_msgs::PGVPose>(m_barcodeGlobalTopic.c_str(), 10);
    nodeMarkerPub     = nh.advertise<visualization_msgs::MarkerArray>("barcode_map", 10);
    dataTimer         = nhp.createTimer(ros::Duration(1.0/m_loopRate), &PGVReader::dataReadCallback, this);
}

/**
 * @/brief  Destructor for the PGVReader
 */
PGVReader::~PGVReader()
{
}


/**
 * @brief  Initializes the parameters from the parameter server
 */
void PGVReader::initializeParameters()
{
    nhp.param<std::string>("port_name", m_portName, "/dev/ttyUSB0");
    nhp.param<std::string>("barcode_local_topic", m_barcodeLocalTopic, "barcode/local");
    nhp.param<std::string>("barcode_global_topic", m_barcodeGlobalTopic, "barcode/global");
    nhp.param<std::string>("csv_file_name", m_csvFileName, "dummy.csv");

    

    nhp.param("baud_rate", m_baudRate, 115200);
    nhp.param<double>("angle_offset", m_angleOffset, 0);
    nhp.param<double>("loop_rate", m_loopRate, 60.0);
ROS_INFO_STREAM("ANGLE  : " << m_angleOffset);
    m_packagePath        = ros::package::getPath("reader_modules");

}

/**
 * @brief  Initializes the connection with the shelf reading module
 */
void PGVReader::initializeConnection()
{
    // // libserial
    p_serialPort = boost::make_shared<LibSerial::SerialPort>();
    p_serialPort->Open(m_portName);
    switch (m_baudRate)
    {
    case 115200:
        p_serialPort->SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        break;
    case 57600:
        p_serialPort->SetBaudRate(LibSerial::BaudRate::BAUD_57600);
        break;
    case 19200:
        p_serialPort->SetBaudRate(LibSerial::BaudRate::BAUD_19200);
        break;
    default:
        ROS_ERROR("Failed to set baudrate. This baudrate is not support yet...");
        // return false;
    }

    p_serialPort->SetParity(LibSerial::Parity::PARITY_EVEN);
    p_serialPort->FlushIOBuffers();
    ros::Duration(0.5).sleep();

    sendTypeToScan();
    p_serialPort->FlushIOBuffers();
}



void PGVReader::sendTypeToScan()
{
    // Straight ahead       =   0xEC, 0x13
    // Follow Left          =   0xE8, 0x17
    // Follow Right         =   0xE4, 0x1B
    // No lane is selected  =   0xE0, 0x1F

    LibSerial::DataBuffer req_type(2);

    req_type[0] = 0xec;
    req_type[1] = ~req_type[0];

    if (p_serialPort->IsOpen())
    {
        p_serialPort->Write(req_type);
        p_serialPort->DrainWriteBuffer();
    }
}

void PGVReader::sendRequestToScan()
{
    LibSerial::DataBuffer req_packet(2);
    req_packet[0] = 0xC8;
    req_packet[1] = ~req_packet[0];

    if (p_serialPort->IsOpen())
    {
        p_serialPort->Write(req_packet);
        p_serialPort->DrainWriteBuffer();
    }
}

void PGVReader::dataReadCallback(const ros::TimerEvent &event)
{
    sendRequestToScan();

    //ROS_INFO("Test");
    /*Receive the packet from PGV100*/
    LibSerial::DataBuffer receivedPacket(21);
    try
    {
        p_serialPort->Read(receivedPacket, 21, 100);
    }
    catch (LibSerial::ReadTimeout e)
    {
        ROS_ERROR_NAMED("PGVReader","[%s] ReadTimeout occurred. check the communication line or device status.", ros::this_node::getName().c_str());
        p_serialPort->FlushIOBuffers();
        return;
    }

    /*Calculate the checksum by XOR [0:19]*/
    int8_t checkSum = receivedPacket[0];
    for (size_t i = 1; i < 20; i++)
    {
        checkSum ^= receivedPacket[i];
    }

    /*Compare with calculated checksum and received checksum*/
    if (checkSum != receivedPacket[20])
    {
        ROS_ERROR_NAMED("PGVReader","[%s] Checksum mismatched...", ros::this_node::getName().c_str());
        p_serialPort->FlushIOBuffers();
        m_checkSumState = false;
        return;
    }
    m_checkSumState = true;

    /*Parse and save the result from received packet*/

    // qrcode_reader_pgv100::QRDetectResult result_msg = qrcode_reader_pgv100::QRDetectResult();

    if (receivedPacket[1] & 0x40)
    {
        m_scanDetected = true;
        // result_msg.is_detected = m_scanDetected;
        //ROS_WARN_NAMED("PGVReader","Bar code detected.");

        int32_t xps = ((int32_t)(receivedPacket[2] & 0x07) << 21 |
                       ((int32_t)(receivedPacket[3] & 0x7f) << 14) |
                       ((int32_t)(receivedPacket[4] & 0x7f) << 7) |
                       (receivedPacket[5] & 0x7f));

        if (xps & 0x40000) // MSB is set, it is negative value
        {
            xps |= 0xff800000;
        }
        // ROS_DEBUG("XPS_: %d", -1 * xps);
        //m_pgvLocalX = (xps / 10000.0);

        // result_msg.pose.position.x = m_pgvPoseX;
        int16_t yps = ((int16_t)(receivedPacket[6] & 0x7f) << 7) |
                      (receivedPacket[7] & 0x7f);

        if (yps & 0x2000) // MSB is set, it is negative value
        {
            yps |= 0xC000;
        }
        // ROS_DEBUG("YPS_: %d", -1 * yps);
	//
	//m_pgvLocalX = (yps / 10000.0);
        //m_pgvLocalY = (-xps / 10000.0);
	//
	m_pgvLocalX = (xps / 10000.0);
        m_pgvLocalY = (yps / 10000.0);

	
	// result_msg.pose.position.y = m_pgvPoseY;
       // ROS_INFO_STREAM("x="<<xps<<"y" <<yps<<"angle offset"<<m_angleOffset);        

        int16_t angle = ((int16_t)(receivedPacket[10] & 0x7f) << 7) |
                      (receivedPacket[11] & 0x7f);

        if (angle & 0x2000) // MSB is set, it is negative value
        {
            angle |= 0xC000;
        }

	double angleDegree = double(angle)/10.0;

	angleDegree = angleDegree - m_angleOffset;


        m_pgvLocalAngle = -1.0f * (angles::normalize_angle((angleDegree) / 180.0f * M_PI));


//    angle = ((angle) / 180.0f * M_PI);
  //   float angle_mod =  fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
    // double a =angle_mod;
    // if (a > M_PI)
      //      a -= 2.0 *M_PI;
      // m_pgvLocalAngle = a;


    //ROS_INFO("angle --> %f ",m_pgvLocalAngle);  
      // ROS_DEBUG("ANG_: %d", angle - m_angleOffset);

        // ROS_INFO("angle: %f , scan_angle %f ", ang,m_pgvAngle);

        // tf2::Quaternion q;
        // q.setRPY(0.0, 0.0, m_pgvLocalAngle);
        // result_msg.pose.orientation.x = q[0];
        // result_msg.pose.orientation.y = q[1];
        // result_msg.pose.orientation.z = q[2];
        // result_msg.pose.orientation.w = q[3];

        uint32_t tag = ((uint32_t)(receivedPacket[14] & 0x07) << 21 |
                        ((uint32_t)(receivedPacket[15] & 0x7f) << 14) |
                        ((uint32_t)(receivedPacket[16] & 0x7f) << 7) |
                        (receivedPacket[17] & 0x7f));
        m_tagId = tag;

        anscer_msgs::PGVPose localMsg,globalMsg;

        localMsg.x      = m_pgvLocalX;
        localMsg.y      = m_pgvLocalY;
        localMsg.yaw    = m_pgvLocalAngle;  
        localMsg.id	= m_tagId;
	localPosePub.publish(localMsg);


        //ROS_INFO_NAMED("PGVReader","Local X : %f  Y : %f  Angle : %f  Tag Id: %d",m_pgvLocalX,m_pgvLocalY,m_pgvLocalAngle,m_tagId);

        BarcodeInfo barcodeInfo = m_barCodeMap[m_tagId];

        m_pgvGlobalX        = barcodeInfo.barcodeToMapX + m_pgvLocalX;
        m_pgvGlobalY        = barcodeInfo.barcodeToMapY + m_pgvLocalY;
        m_pgvGlobalAngle    = m_pgvLocalAngle;

        globalMsg.x      = m_pgvGlobalX;
        globalMsg.y      = m_pgvGlobalY;
        globalMsg.yaw    = m_pgvGlobalAngle;  
	globalMsg.id	 = m_tagId;
        globalPosePub.publish(globalMsg);

        // ROS_WARN_NAMED("PGVReader","Global X : %f  Y : %f  Angle : %f",m_pgvGlobalX,m_pgvGlobalY,m_pgvGlobalAngle);

    }
    

    mapVisualization();
}
void PGVReader::mapVisualization()
{
    m_markerArray.markers.clear();
    m_markerArray.markers.resize(0);
    
    /*Visualization of the barcode map generated*/
    for (auto it = m_barCodeMap.begin(); it != m_barCodeMap.end(); it++)
    {
        visualization_msgs::Marker marker;

        int currentMarker = it->first;

        if(currentMarker == m_tagId)
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }

        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();

        marker.ns = "node_" + std::to_string(it->first);
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.color.a = 1.0; // Don't forget to set the alpha!

        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.pose.position.x = it->second.barcodeToMapX;
        marker.pose.position.y = it->second.barcodeToMapY;
        m_markerArray.markers.push_back(marker);
        //ROS_INFO("Id  : %d   Map-X  : %f   Map-Y  : %f", it->first, marker.pose.position.x, marker.pose.position.y);
    }
    m_tagId = 0;


    nodeMarkerPub.publish(m_markerArray);
}

// /**
// * @brief Reads the data from barcode db
// */
// void PGVReader::readBarCodeInfoFromDB()
// {
//     sqlite3 *DB;
//     sqlite3_stmt *stmt;
//     char *messaggeError;
//     std::string filePath = m_packagePath + "/data/BarcodeInfo.db";
//     ROS_WARN("Read path : %s",filePath.c_str());
//     int exit = sqlite3_open(filePath.c_str(), &DB);
//     std::string query = "SELECT * FROM  BARCODEINFO";

//     int id;
//     double x, y;
    
//     sqlite3_prepare_v2(DB, query.c_str(), -1, &stmt, nullptr);
//     while (sqlite3_step(stmt) != SQLITE_DONE)
//     {
//         id     = sqlite3_column_int(stmt, 0);
//         x      = sqlite3_column_double(stmt, 1);
//         y      = sqlite3_column_double(stmt, 2);
//         // yaw    = sqlite3_column_double(stmt, 3);

//         // ROS_INFO("Bar code info : %d   %f   %f ",id,x,y);

//         BarcodeInfo barCodeInfo;

//         barCodeInfo.barcodeToMapX  = x;
//         barCodeInfo.barcodeToMapY  = y;
//         m_barCodeMap[id]  = barCodeInfo;
//     }

    
//     /*Visualization of the barcode map generated*/
//     for (auto it = m_barCodeMap.begin(); it != m_barCodeMap.end(); it++)
//     {
//         visualization_msgs::Marker marker;
//         marker.header.frame_id = "map";
//         marker.header.stamp = ros::Time(0);
//         marker.ns = "node_" + std::to_string(it->first);
//         marker.action = visualization_msgs::Marker::ADD;
//         marker.pose.orientation.w = 1.0;
//         marker.id = 1;
//         marker.type = visualization_msgs::Marker::SPHERE;
//         marker.color.a = 1.0; // Don't forget to set the alpha!
//         marker.color.r = 0.0;
//         marker.color.g = 1.0;
//         marker.color.b = 0.0;
//         marker.scale.x = 0.25;
//         marker.scale.y = 0.25;
//         marker.scale.z = 0.25;
//         marker.pose.position.x = it->second.barcodeToMapX;
//         marker.pose.position.y = it->second.barcodeToMapY;
//         m_markerArray.markers.push_back(marker);
//         ROS_INFO("Id  : %d   Map-X  : %f   Map-Y  : %f", it->first, it->second.barcodeToMapX, it->second.barcodeToMapY);
//     }
//     sqlite3_finalize(stmt);
//     sqlite3_close(DB);
// }

/**
 * @brief Reads the data from barcode to map relation from CSV file
 */
void PGVReader::readBarCodeInfoFromCSV()
{
    std::string filePath = m_packagePath + "/data/" + m_csvFileName;
    FILE *fp = fopen(filePath.c_str(), "r");

    if (!fp)
    {
        ROS_ERROR_NAMED("PGVReader", "Unable to open the CSV file");
    }
    else
    {
        ROS_INFO_NAMED("PGVReader","Reading data from csv");
        // Here we have taken size of
        // array 1024 you can modify it
        char buffer[1024];

        int row = 0;
        int column = 0;
        
        while (fgets(buffer, 1024, fp))
        {
            column = 0;
            row++;
            if (row == 1)
                continue;

            int buffer_size = sizeof(buffer) / sizeof(char);
            std::string s_buffer = convertToString(buffer, buffer_size);

            std::stringstream ss(s_buffer);
            int columnNumber = 0;
            BarcodeInfo barCodeInfo;
            while (ss.good() && (columnNumber <3))
            {
                std::string substr;
                getline(ss, substr, ',');
                // std::cout << substr << std::endl;

                if(columnNumber == 0)
                {
                    barCodeInfo.barcodeToMapX  = std::stod(substr)*1;
                }
                else if(columnNumber == 1)
                {
                    barCodeInfo.barcodeToMapY  = std::stod(substr)*1;
                }     
                else if(columnNumber == 2)
                {
                    int id = std::stoi(substr);
                    m_barCodeMap[id]  = barCodeInfo;
                }     
                columnNumber++;

            }
            //printf("\n");
        }

        fclose(fp);
    }
}


/**
 * @brief Converts the char array to string
 * @param a :char array and size : the size of the char array
 * @return converted char array in string format
 */
std::string PGVReader::convertToString(char *a, int size)
{
    int i;
    std::string s = "";
    for (i = 0; i < size; i++)
    {
        s = s + a[i];
    }
    return s;
}


/*
 * @brief Main function
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pgv_reader");
    PGVReader pgvReader;

    ros::Rate r(100);
    while (ros::ok())
    {

        r.sleep();
        ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}
