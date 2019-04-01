/*
 * ControlsManager.cpp
 *
 *  Created on: 2019-01-09
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "ControlsManager.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <chrono>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string RightMotorName = "rotationalMotorRight";
const std::string LeftMotorName = "rotationalMotorLeft";
const std::string RightEncoderName = "positionSensorRight";
const std::string LeftEncoderName = "positionSensorLeft";
const std::string CompassName = "compass";

constexpr int EncoderSamplingPeriod = 10;
constexpr int EncoderResolution = 1024; //Number of ticks
constexpr float WheelDiameter = 0.05;
constexpr float WheelBase = 0.112;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

ControlsManager::ControlsManager()
{
    this->_rightEncoderData = 0;
    this->_leftEncoderData = 0;
    this->_odometry = 0.0;
    this->_turnRate = 0.0;
    this->_speed = 0.0;
    this->_time = 0.0;
    this->_robot= nullptr;
    this->_leftMotor = nullptr;
    this->_rightMotor = nullptr;
    this->_leftEncoder = nullptr;
    this->_rightEncoder = nullptr;
    this->_compass = nullptr;
    this->_timeChange = 0.0;
    this->_leftEncoderChange = 0.0;
    this->_rightEncoderChange = 0.0;
    this->_initialHeading = 0.0;
    this->_initalHeadingSet = false;

    this->_heading = 0.0;
}

ControlsManager::~ControlsManager()
{
    //let SystemInterface delete robot object
}

bool
ControlsManager::Initialize(webots::Robot* robot)
{
    this->_time = getTimestamp();

    this->_robot = robot;

    this->_rightMotor = _robot->getMotor(RightMotorName);
    this->_leftMotor = _robot->getMotor(LeftMotorName);
    this->_rightEncoder = _robot->getPositionSensor(RightEncoderName);
    this->_leftEncoder = _robot->getPositionSensor(LeftEncoderName);
    this->_compass = _robot->getCompass(CompassName);

    if((!_rightMotor) || (!_leftMotor))
    {
        std::cout<<"[ERROR]: Motors Could not be initialized"<<std::endl;
        return false;
    }

    if(_rightEncoder && _leftEncoder)
    {
        _rightEncoder->enable(EncoderSamplingPeriod);
        _leftEncoder->enable(EncoderSamplingPeriod);
    }

    if(_compass)
    {
        _compass->enable(EncoderSamplingPeriod);
    }

    SetLeftMotorVelocity(0.0);
    SetRightMotorVelocity(0.0);

    return true;
}

void
ControlsManager::PopulateData()
{
    if(_compass)
    {
        if(!_initalHeadingSet)
        {
            const double* values = _compass->getValues();
            _initialHeading = getBearing(values[0], values[2]);
            _initalHeadingSet = true;
        }
    }
    if(_rightEncoder && _leftEncoder)
    {
        double updatedTime = getTimestamp();
        float updatedRightEncoder = ((_rightEncoder->getValue() / (2*M_PI)) * EncoderResolution);
        float updatedLeftEncoder = ((_leftEncoder->getValue() / (2*M_PI)) * EncoderResolution);

        int rightDiff = updatedRightEncoder - _rightEncoderData;
        int leftDiff = updatedLeftEncoder - _leftEncoderData;
        double timeDiff = (updatedTime - _time);
        _timeChange = timeDiff;

        this->_rightEncoderChange = ((float)rightDiff/EncoderResolution) * 2.0 * M_PI;
        this->_leftEncoderChange = ((float)leftDiff/EncoderResolution) * 2.0 * M_PI;

        this->_odometry += (rightDiff + leftDiff)/2.0 * (2.0*M_PI/EncoderResolution) * (WheelDiameter/2.0);
        this->_turnRate = ( ((leftDiff - rightDiff)/2.0) * (2.0*M_PI/EncoderResolution)  * (WheelDiameter/2.0)) / ((WheelBase/2.0)*timeDiff);
        this->_speed = (fabs(rightDiff + leftDiff)/2.0 * (2.0*M_PI/EncoderResolution) * (WheelDiameter/2.0)) / timeDiff;

        this->_rightEncoderData = updatedRightEncoder;
        this->_leftEncoderData = updatedLeftEncoder;
        this->_time = updatedTime;

//        _heading += ( ((leftDiff - rightDiff)/2.0) * (2.0*M_PI/EncoderResolution)  * (WheelDiameter/2.0)) / (WheelBase/2.0);
//        if(_heading > (M_PI)) _heading -= (M_PI);
//        else if(_heading < -M_PI) _heading += M_PI;

        const double* values = _compass->getValues();
        _heading = getBearing(values[0], values[2]);
    }
}

double
ControlsManager::getBearing(double northx, double northz)
{
    double rad = atan2(northx, northz);
    double bearing = (rad - 1.5708) - _initialHeading;
    if (bearing < -M_PI) bearing += M_PI;
    if(bearing > M_PI) bearing -= M_PI;
    return bearing;
}

void
ControlsManager::SetRightMotorVelocity(float velocity)
{
    if(_rightMotor)
    {
        _rightMotor->setPosition(INFINITY);
        _rightMotor->setVelocity(velocity);
    }
    else
    {
        std::cout<<"[ERROR]: Right Motor is not initialized"<<std::endl;
    }
}

void
ControlsManager::SetLeftMotorVelocity(float velocity)
{
    if(_leftMotor)
    {
        _leftMotor->setPosition(INFINITY);
        _leftMotor->setVelocity(velocity);
    }
    else
    {
        std::cout<<"[ERROR]: Left Motor is not initialized"<<std::endl;
    }
}

double
ControlsManager::getTimestamp()
{
    return (double)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}



