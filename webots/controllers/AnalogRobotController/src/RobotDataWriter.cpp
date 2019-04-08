/*
 * RobotDataWriter.cpp
 *
 *  Created on: 2019-01-11
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "RobotDataWriter.hpp"

#include <iostream>
#include <cmath>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr float RobotSpeed = 10.0;
constexpr float RobotTurningRate = 5.0;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

RobotDataWriter::RobotDataWriter()
{
    if(_fileWriter.is_open())
    {
        CloseFile();
    }
}

void
RobotDataWriter::Initialize(std::string filename)
{
    this->_fileWriter.open(filename, std::ofstream::out | std::ofstream::app);
}

void
RobotDataWriter::CloseFile()
{
    this->_fileWriter.close();
}

void
RobotDataWriter::WriteData(SensorManager &sensors, ControlsManager &controls, float radiusToGoal, float angleToGoal, keypadDirection direction)
{
    if(_fileWriter.is_open())
    {
        _fileWriter << 1.0 - (sensors.GetLeftDistanceSensor().distance / 100.0) <<",";
        _fileWriter << 1.0 - (sensors.GetCenterDistanceSensor().distance / 100.0) <<",";
        _fileWriter << 1.0 - (sensors.GetRightDistanceSensor().distance / 100.0) <<",";
        _fileWriter << radiusToGoal <<",";
        _fileWriter << (angleToGoal / M_PI) <<",";
        _fileWriter << direction <<std::endl;
    }
}





