/*
 * DataCollectorMain.cpp
 *
 *  Created on: 2019-04-07
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "DataCollector.hpp"

#include <string>
#include <iostream>

/****************************************
 * NAMESPACE
 ****************************************/

using namespace ANN;

/****************************************
 * CONSTANTS
 ****************************************/

std::string filename = "AnalogRobotData.csv";

/****************************************
 * Main
 ****************************************/

int main (void)
{
    DataCollector collector;

    collector.Initialize(filename);

    std::cout<<"start"<<std::endl;

    while(!collector.GetQuitFlag())
    {
        collector.GenerateSituation();

        std::cout<<"key pad direction: "<<(int)collector.GetDirection()<<std::endl;

        collector.WriteData(collector.GetLeftSensor(), collector.GetCenterSensor(), collector.GetRightSensor(), collector.GetRadius(), collector.GetAngle(), collector.GetDirection());
    }

    collector.CloseFile();
    std::cout<<"data collector utility ended"<<std::endl;
}


