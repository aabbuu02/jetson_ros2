/**
 * @file pgv_reader.cpp
 * @author Abubakarsiddiq Navid shaikh
 * @date 2024-10-05
 * @brief Auto-generated author information
 */

#include "reader_modules/pgv_reader.h"
/**
 * @brief Constructor for the PGVReader
 */
PGVReader::PGVReader() : Node("pgv_reader")
{
    initializeParameters();
    initializeConnection();
    readBarCodeInfoFromCSV();
    
    localPosePub = this->create_publisher<anscer_msgs::msg::PGVPose>(
        m_barcodeLocalTopic, 10);
    globalPosePub = this->create_publisher<anscer_msgs::msg::PGVPose>(
        m_barcodeGlobalTopic, 10);
    nodeMarkerPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "barcode_map", 10);
    
    dataTimer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / m_loopRate)),
        std::bind(&PGVReader::dataReadCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "PGVReader node initialized");
}

/**
 * @brief Destructor for the PGVReader
 */
PGVReader::~PGVReader()
{
    if (p_serialPort && p_serialPort->IsOpen())
    {
        p_serialPort->Close();
    }
}

/**
 * @brief Initializes the parameters from the parameter server
 */
void PGVReader::initializeParameters()
{
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("barcode_local_topic", "barcode/local");
    this->declare_parameter<std::string>("barcode_global_topic", "barcode/global");
    this->declare_parameter<std::string>("csv_file_name", "dummy.csv");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<double>("angle_offset", 0.0);
    this->declare_parameter<double>("loop_rate", 60.0);

    this->get_parameter("port_name", m_portName);
    this->get_parameter("barcode_local_topic", m_barcodeLocalTopic);
    this->get_parameter("barcode_global_topic", m_barcodeGlobalTopic);
    this->get_parameter("csv_file_name", m_csvFileName);
    this->get_parameter("baud_rate", m_baudRate);
    this->get_parameter("angle_offset", m_angleOffset);
    this->get_parameter("loop_rate", m_loopRate);

    RCLCPP_INFO(this->get_logger(), "ANGLE OFFSET: %f", m_angleOffset);

    try {
        m_packagePath = ament_index_cpp::get_package_share_directory("reader_modules");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get package path: %s", e.what());
        m_packagePath = "";
    }
}

/**
 * @brief Initializes the connection with the PGV reading module
 */
void PGVReader::initializeConnection()
{
    p_serialPort = std::make_shared<LibSerial::SerialPort>();
    
    try {
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
            RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate. This baudrate is not supported yet...");
            return;
        }

        p_serialPort->SetParity(LibSerial::Parity::PARITY_EVEN);
        p_serialPort->FlushIOBuffers();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        sendTypeToScan();
        p_serialPort->FlushIOBuffers();
        
        RCLCPP_INFO(this->get_logger(), "Serial port connected successfully");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    }
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

double PGVReader::normalize_angle(double angle)
{
    double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        result += 2.0 * M_PI;
    return result - M_PI;
}

void PGVReader::dataReadCallback()
{
    sendRequestToScan();

    // Receive the packet from PGV100
    LibSerial::DataBuffer receivedPacket(21);
    try
    {
        p_serialPort->Read(receivedPacket, 21, 100);
    }
    catch (LibSerial::ReadTimeout& e)
    {
        RCLCPP_ERROR(this->get_logger(), "ReadTimeout occurred. Check the communication line or device status.");
        p_serialPort->FlushIOBuffers();
        return;
    }

    // Calculate the checksum by XOR [0:19]
    int8_t checkSum = receivedPacket[0];
    for (size_t i = 1; i < 20; i++)
    {
        checkSum ^= receivedPacket[i];
    }

    // Compare with calculated checksum and received checksum
    if (checkSum != receivedPacket[20])
    {
        RCLCPP_ERROR(this->get_logger(), "Checksum mismatched...");
        p_serialPort->FlushIOBuffers();
        m_checkSumState = false;
        return;
    }
    m_checkSumState = true;

    // Parse and save the result from received packet
    if (receivedPacket[1] & 0x40)
    {
        m_scanDetected = true;

        int32_t xps = ((int32_t)(receivedPacket[2] & 0x07) << 21 |
                       ((int32_t)(receivedPacket[3] & 0x7f) << 14) |
                       ((int32_t)(receivedPacket[4] & 0x7f) << 7) |
                       (receivedPacket[5] & 0x7f));

        if (xps & 0x40000) // MSB is set, it is negative value
        {
            xps |= 0xff800000;
        }

        int16_t yps = ((int16_t)(receivedPacket[6] & 0x7f) << 7) |
                      (receivedPacket[7] & 0x7f);

        if (yps & 0x2000) // MSB is set, it is negative value
        {
            yps |= 0xC000;
        }

        m_pgvLocalX = (xps / 10000.0);
        m_pgvLocalY = (yps / 10000.0);

        int16_t angle = ((int16_t)(receivedPacket[10] & 0x7f) << 7) |
                      (receivedPacket[11] & 0x7f);

        if (angle & 0x2000) // MSB is set, it is negative value
        {
            angle |= 0xC000;
        }

        double angleDegree = double(angle) / 10.0;
        angleDegree = angleDegree - m_angleOffset;
        m_pgvLocalAngle = -1.0f * normalize_angle((angleDegree) / 180.0f * M_PI);

        uint32_t tag = ((uint32_t)(receivedPacket[14] & 0x07) << 21 |
                        ((uint32_t)(receivedPacket[15] & 0x7f) << 14) |
                        ((uint32_t)(receivedPacket[16] & 0x7f) << 7) |
                        (receivedPacket[17] & 0x7f));
        m_tagId = tag;

        anscer_msgs::msg::PGVPose localMsg, globalMsg;

        localMsg.x = m_pgvLocalX;
        localMsg.y = m_pgvLocalY;
        localMsg.yaw = m_pgvLocalAngle;
        localMsg.id = m_tagId;
        localPosePub->publish(localMsg);

        BarcodeInfo barcodeInfo = m_barCodeMap[m_tagId];

        m_pgvGlobalX = barcodeInfo.barcodeToMapX + m_pgvLocalX;
        m_pgvGlobalY = barcodeInfo.barcodeToMapY + m_pgvLocalY;
        m_pgvGlobalAngle = m_pgvLocalAngle;

        globalMsg.x = m_pgvGlobalX;
        globalMsg.y = m_pgvGlobalY;
        globalMsg.yaw = m_pgvGlobalAngle;
        globalMsg.id = m_tagId;
        globalPosePub->publish(globalMsg);
    }

    mapVisualization();
}

void PGVReader::mapVisualization()
{
    m_markerArray.markers.clear();

    // Visualization of the barcode map generated
    for (auto it = m_barCodeMap.begin(); it != m_barCodeMap.end(); it++)
    {
        visualization_msgs::msg::Marker marker;

        int currentMarker = it->first;

        if (currentMarker == m_tagId)
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
        marker.header.stamp = this->now();

        marker.ns = "node_" + std::to_string(it->first);
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.color.a = 1.0;

        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.pose.position.x = it->second.barcodeToMapX;
        marker.pose.position.y = it->second.barcodeToMapY;
        m_markerArray.markers.push_back(marker);
    }
    m_tagId = 0;

    nodeMarkerPub->publish(m_markerArray);
}

/**
 * @brief Reads the data from barcode to map relation from CSV file
 */
void PGVReader::readBarCodeInfoFromCSV()
{
    std::string filePath = m_packagePath + "/data/" + m_csvFileName;
    FILE *fp = fopen(filePath.c_str(), "r");

    if (!fp)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to open the CSV file: %s", filePath.c_str());
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Reading data from csv");
        char buffer[1024];

        int row = 0;

        while (fgets(buffer, 1024, fp))
        {
            row++;
            if (row == 1)
                continue;

            int buffer_size = sizeof(buffer) / sizeof(char);
            std::string s_buffer = convertToString(buffer, buffer_size);

            std::stringstream ss(s_buffer);
            int columnNumber = 0;
            BarcodeInfo barCodeInfo;
            
            while (ss.good() && (columnNumber < 3))
            {
                std::string substr;
                getline(ss, substr, ',');

                if (columnNumber == 0)
                {
                    barCodeInfo.barcodeToMapX = std::stod(substr) * 1;
                }
                else if (columnNumber == 1)
                {
                    barCodeInfo.barcodeToMapY = std::stod(substr) * 1;
                }
                else if (columnNumber == 2)
                {
                    int id = std::stoi(substr);
                    m_barCodeMap[id] = barCodeInfo;
                }
                columnNumber++;
            }
        }

        fclose(fp);
    }
}

/**
 * @brief Converts the char array to string
 */
std::string PGVReader::convertToString(char *a, int size)
{
    std::string s = "";
    for (int i = 0; i < size; i++)
    {
        s = s + a[i];
    }
    return s;
}

/**
 * @brief Main function
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PGVReader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
