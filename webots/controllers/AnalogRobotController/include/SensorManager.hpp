/*
 * SensorManager.hpp
 *
 *  Created on: 2019-01-09
 *      Author: liam
 */

#ifndef SENSORMANAGER_HPP_
#define SENSORMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

#include <vector>

/****************************************
 * STRUCTS
 ****************************************/

struct distanceSensor
{
    distanceSensor() : angleOffset(0.0), distance(0.0) {};

    float angleOffset; //radians
    float distance; //meters
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class SensorManager
{

public:

    SensorManager();
    ~SensorManager();

    bool Initialize(webots::Robot* robot);
    void PopulateData();

    distanceSensor GetCenterDistanceSensor() const {return _centerData;};
    distanceSensor GetRightDistanceSensor() const {return _rightData;};
    distanceSensor GetLeftDistanceSensor() const {return _leftData;};

private:

    distanceSensor _centerData;
    distanceSensor _leftData;
    distanceSensor _rightData;

    webots::Robot* _robot;
    webots::DistanceSensor *_centerSensor;
    webots::DistanceSensor *_rightSensor;
    webots::DistanceSensor *_leftSensor;
};



#endif /* SENSORMANAGER_HPP_ */
