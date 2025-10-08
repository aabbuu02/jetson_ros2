#ifndef ODRIVE_CONTROLLER_H
#define ODRIVE_CONTROLLER_H


#include "AHighLevelController.h"

#include "ODriveDevice.h"


#define ODRIVESERIALNUMBER "207D39694D4D"


class ODriveController: public AHighLevelController
{

public:
    ODriveController(ros::NodeHandle &nh);
    virtual ~ODriveController(){}

    void writeCommands(writeParameters &writeParams) override;
    void readCommands(readParameters &readParam) override;


private:

    ros::NodeHandle nhp;

    /*Member Functions*/
    void initialize(ros::NodeHandle &nh);
    void initializeParameters();
    void connectToDevice();
    void watchDogTimer();

    /*Member variables*/
    int m_baudRate;
    std::string m_odriveSerialNumber;
    CommunicationSerial* p_communicationSerial;
    std::vector<std::shared_ptr<ODriveDevice> >p_ODriveVector;
};

#endif
