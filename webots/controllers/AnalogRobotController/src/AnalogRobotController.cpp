/*
 * AnalogRobotController.cpp
 *
 *  Created on: 2019-01-08
 *      Author: liam
 */

#include "SystemInterface.hpp"

#include <webots/Robot.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    SystemInterface interface;

    interface.Initialize(7.83, -5.65);
    interface.Start();
    return 0;
}



