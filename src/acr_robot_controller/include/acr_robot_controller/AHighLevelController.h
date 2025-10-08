#ifndef AHIGHLEVELCONTROLLER_H
#define AHIGHLEVELCONTROLLER_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <bitset>


#include "Utilities.h"


class AHighLevelController
{
public:
    // AHighLevelController(){}
    virtual ~AHighLevelController(){}
    virtual void writeCommands(writeParameters &writeParams) = 0;
    virtual void readCommands(readParameters &readParam) = 0;
};

#endif
