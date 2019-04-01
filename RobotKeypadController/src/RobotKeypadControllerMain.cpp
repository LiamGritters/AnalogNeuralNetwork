/*
 * RobotKeypadControllerMain.cpp
 *
 *  Created on: 2019-01-10
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "RobotKeypadController.hpp"

#include "keypadcontroller_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string channel = "CONTROLLER";

/****************************************
 * Main
 ****************************************/

int main()
{
    RobotKeypadController controller;

    controller.Start();

    lcm::LCM lcm;

    if(!lcm.good()) return 1;

    ANNLCM::keypadcontroller_t msg;
    msg.enabled = true;
    msg.name = "controller";

    while(controller.IsRunning())
    {
        msg.direction = controller.GetDirection();

        lcm.publish(channel, &msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    msg.enabled = false;
    lcm.publish(channel, &msg);
}



