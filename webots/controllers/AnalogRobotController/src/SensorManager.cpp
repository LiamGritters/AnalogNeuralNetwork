/*
 * SensorManager.cpp
 *
 *  Created on: 2019-01-09
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "SensorManager.hpp"

#include <iostream>
#include <string>

/****************************************
 * CONSTANTS
 ****************************************/

// Webots Sensor Names -> set in the simulator
const std::string CenterSensorName = "distanceSensorCenter";
const std::string LeftSensorName = "distanceSensorLeft";
const std::string RightSensorName = "distanceSensorRight";

// Sensors Sampling Periods (Hz)
constexpr int DistanceSensorPeriod = 10;

constexpr float AngleOffsetMagnitude = 0.6981317;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

SensorManager::SensorManager()
{
    this->_robot = nullptr;
    this->_centerSensor = nullptr;
    this->_rightSensor = nullptr;
    this->_leftSensor = nullptr;
};

SensorManager::~SensorManager()
{
    //let WebotsSystemInterface delete robot object
}

bool
SensorManager::Initialize(webots::Robot* robot)
{
    this->_robot = robot;

    //Initialize all the sensors
    this->_centerSensor = _robot->getDistanceSensor(CenterSensorName);
    this->_leftSensor = _robot->getDistanceSensor(LeftSensorName);
    this->_rightSensor = _robot->getDistanceSensor(RightSensorName);

    if((!_centerSensor) && (!_leftSensor) && (!_rightSensor))
    {
        return false;
    }

    _centerSensor->enable(DistanceSensorPeriod);
    _rightSensor->enable(DistanceSensorPeriod);
    _leftSensor->enable(DistanceSensorPeriod);

    _leftData.angleOffset = -AngleOffsetMagnitude;
    _rightData.angleOffset = AngleOffsetMagnitude;

    return true;
}

void
SensorManager::PopulateData()
{
    _centerData.distance = (float)(_centerSensor->getValue()/10.0);
    _rightData.distance = (float)(_rightSensor->getValue()/10.0);
    _leftData.distance = (float)(_leftSensor->getValue()/10.0);
}



