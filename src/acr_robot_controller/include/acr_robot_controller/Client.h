#ifndef CLIENT_H
#define CLIENT_H


#include "RoboteqController.h"




class Client {
public:
    Client(ros::NodeHandle &nh,int type)
    {
        ROS_INFO_NAMED("Client","Client costructor called with type %d", type);
        if (type == ROBOTEQ_DEVICE)
        {
            p_highLevelController = new RoboteqController(nh);
        }
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
