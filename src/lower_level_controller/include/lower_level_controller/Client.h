#ifndef CLIENT_H
#define CLIENT_H

#include "ModbusController.h"
//#include "ModbusRoboteqController.h"
//#include "ODriveController.h"
//#include "RainbowController.h"



class Client {
public:
    Client(ros::NodeHandle &nh,int type)
    {
        ROS_INFO_NAMED("Client","Client costructor called with type %d", type);
        if (type == MODBUS_DEVICE)
        {
            p_highLevelController = new ModbusController(nh);
        }
        /*
        else if (type == MODBUS_ROBOTEQ_DEVICE)
        {
            p_highLevelController = new ModbusRoboteqController(nh);
        }
        // else if (type == ODRIVE_DEVICE)
        // {
        //     p_highLevelController = new ODriveController(nh);
        // }
        else if (type == RAINBOW_DEVICE)
        {
            p_highLevelController = new RainbowController(nh);
        }
        */
        else
        {
            p_highLevelController = NULL;
            ROS_FATAL_NAMED("Client","Please check the controller type selected");
            exit(0);
        }
            
    }
 
    ~Client()
    {
        if (p_highLevelController) {
            delete[] p_highLevelController;
            p_highLevelController = NULL;
        }
    }
 
    AHighLevelController* getHighLevelController() { return p_highLevelController; }
 
private:
    AHighLevelController* p_highLevelController;
};

#endif
