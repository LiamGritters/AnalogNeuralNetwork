/*
 * ControlsManager.hpp
 *
 *  Created on: 2019-01-09
 *      Author: liam
 */

#ifndef CONTROLSMANAGER_HPP_
#define CONTROLSMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>

/****************************************
 * STRUCTS
 ****************************************/

/****************************************
 * CLASS DEFINITION
 ****************************************/

class ControlsManager
{

public:

    ControlsManager();
    ~ControlsManager();

    bool Initialize(webots::Robot* robot);
    void PopulateData();

    float GetOdometry() const {return _odometry;};
    float GetTurningRate() const {return _turnRate;};
    float GetTime() const {return _time;};
    float GetChangeInTime() const {return _timeChange;};

    float GetHeading() const {return _heading;};

    float GetRightEncoderChange () const {return _rightEncoderChange;};
    float GetLeftEncoderChange () const {return _leftEncoderChange;};

    void SetLeftMotorVelocity(float velocity);
    void SetRightMotorVelocity(float velocity);

private:

    double getTimestamp();
    double getBearing(double northx, double northz);

private:

    int _rightEncoderData;
    int _leftEncoderData;

    float _rightEncoderChange;
    float _leftEncoderChange;

    float _odometry;
    float _turnRate;
    float _speed;

    float _heading;
    float _initialHeading;
    bool _initalHeadingSet;

    double _time;
    double _timeChange;

    webots::Robot* _robot;
    webots::Motor* _rightMotor;
    webots::Motor* _leftMotor;
    webots::PositionSensor* _rightEncoder;
    webots::PositionSensor* _leftEncoder;
    webots::Compass* _compass;
};




#endif /* CONTROLSMANAGER_HPP_ */
