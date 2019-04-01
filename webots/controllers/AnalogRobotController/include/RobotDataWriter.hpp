/*
 * RobotDataWriter.hpp
 *
 *  Created on: 2019-01-11
 *      Author: liam
 */

#ifndef ROBOTDATAWRITER_HPP_
#define ROBOTDATAWRITER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "SensorManager.hpp"
#include "ControlsManager.hpp"
#include "RobotKeypadController.hpp"

#include <fstream>
#include <string>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class RobotDataWriter
{

public:

    RobotDataWriter();

    void Initialize(std::string filename);
    void WriteData(SensorManager &sensors, ControlsManager &controls, float radiusToGoal, float angleToGoal, keypadDirection direction);
    void CloseFile();

private:

    std::ofstream _fileWriter;
};



#endif /* ROBOTDATAWRITER_HPP_ */
