/*
 * SystemInterface.cpp
 *
 *  Created on: 2019-01-08
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "SystemInterface.hpp"
#include "RobotKeypadController.hpp"
#include "neuralnetworkdata_t.hpp"

#include <iostream>
#include <chrono>
#include <cmath>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr float RobotSpeed = 5.0;
constexpr float RobotTurningRate = 2.5;

const std::string NeuralNetworkChannel = "NEURAL";

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

SystemInterface::SystemInterface()
{
    this->_timeStep = 0;
    this->_rightSpeed = 0;
    this->_leftSpeed = 0;
    this->_isRunning = false;
    this->_robot = nullptr;
    this->_webotsFlag = 0;
    this->_radiusToGoal = 0.0;
    this->_angleToGoal = 0.0;
    this->_odometry = 0.0;
    this->_heading = 0.0;

    this->_goalX = 0.0;
    this->_goalY = 0.0;
    this->_xr = 0.0;
    this->_yr = 0.0;

    this->_rightOrLeftFlag = false;
    this->_initialRadiusToGoal = 0.0;
}

SystemInterface::~SystemInterface()
{
    Stop();
    if(_robot)
    {
        delete _robot;
    }
}

bool
SystemInterface::Initialize(float radius, float angle)
{
    this->_webotsFlag = 0;
    this->_robot = new webots::Robot();
    this->_timeStep = _robot->getBasicTimeStep();

    this->_sensors.Initialize(_robot);
    this->_controls.Initialize(_robot);

    this->_initialRadiusToGoal = radius;
    this->_radiusToGoal = radius;
    this->_angleToGoal = angle *(M_PI/180.0);

    this->_goalX = radius * sin(_angleToGoal);
    this->_goalY = radius * cos(_angleToGoal);

    this->_writer.Initialize("AnalogRobotData.csv");

    this->_odometry = _controls.GetOdometry();

    return setupNetwork();
}

void
SystemInterface::Start()
{
    this->_isRunning = true;
    this->_thread = std::thread(SystemInterface::robotProcess, this);

    this->_publishThread = std::thread(SystemInterface::publishData, &_lcm, this, &_sensors);

    cv::namedWindow( "Display window", CV_WINDOW_NORMAL );// Create a window for display.

    while(_webotsFlag != -1)
    {
        _lcm.handle();

        cv::Mat image(600, 400, CV_8U, cv::Scalar(0));

        const float leftx = 100 + _sensors.GetLeftDistanceSensor().distance*5.0*sin(-5.0 * M_PI/180.0);
        const float lefty = 600 - _sensors.GetLeftDistanceSensor().distance*5.0*cos(-5.0 * M_PI/180.0);
        const float rightx = 300 + _sensors.GetRightDistanceSensor().distance*5.0*sin(5.0 * M_PI/180.0);
        const float righty = 600 - _sensors.GetRightDistanceSensor().distance*5.0*cos(5.0 * M_PI/180.0);
        const float center = 600 - _sensors.GetCenterDistanceSensor().distance*5.0;

        cv::line(image, cv::Point(100,600), cv::Point(leftx,lefty),cv::Scalar(255), 3, 8, 0);
        cv::line(image, cv::Point(300,600), cv::Point(rightx,righty),cv::Scalar(255), 3, 8, 0);
        cv::line(image, cv::Point(200,600), cv::Point(200,center),cv::Scalar(255), 2, 3, 0);

        const float destx = 200*sin(-_angleToGoal) + 200;
        const float desty = (_radiusToGoal*600*cos(_angleToGoal) > 600) ? 0 : 600 - _radiusToGoal*600*cos(_angleToGoal);
        cv::circle(image, cv::Point(destx,desty),10, cv::Scalar(125), 10, 8, 0);

        cv::imshow( "Display window", image );
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void
SystemInterface::Stop()
{
    _isRunning = false;
    if(_thread.joinable())
    {
        _thread.join();
    }
    if(_publishThread.joinable())
    {
        _publishThread.join();
    }
}

bool
SystemInterface::setupNetwork()
{
    if(_lcm.good())
    {
        _lcm.subscribe("CONTROLLER", &SystemInterface::handleMessage, this);
        return true;
    }
    return false;
}

void
SystemInterface::robotProcess(SystemInterface *sysInterface)
{
    while(sysInterface->_isRunning)
    {
        sysInterface->_webotsFlag = sysInterface->_robot->step(sysInterface->_timeStep);

        sysInterface->_sensors.PopulateData();
        sysInterface->_controls.PopulateData();

        sysInterface->_controls.SetLeftMotorVelocity(sysInterface->_leftSpeed);
        sysInterface->_controls.SetRightMotorVelocity(sysInterface->_rightSpeed);
    }
}

void
SystemInterface::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const ANNLCM::keypadcontroller_t *msg)
{
    if(msg->enabled)
    {
        switch(msg->direction)
        {
            case FORWARD:
                this->_leftSpeed = RobotSpeed;
                this->_rightSpeed = RobotSpeed;
                _rightOrLeftFlag = false;
                break;
            case LEFT:
                this->_leftSpeed = RobotTurningRate;
                this->_rightSpeed = -RobotTurningRate;
                _rightOrLeftFlag = true;
                break;
            case RIGHT:
                this->_leftSpeed = -RobotTurningRate;
                this->_rightSpeed = RobotTurningRate;
                _rightOrLeftFlag = true;
                break;
            case STOP:
                this->_leftSpeed = 0;
                this->_rightSpeed = 0;
                _rightOrLeftFlag = false;
        }

        float newOdometry =  _controls.GetOdometry();
        const float odometryChange = newOdometry - _odometry;
        _odometry = newOdometry;

        _heading = _controls.GetHeading();

        float angle = atan2((_goalX - _xr), (_goalY - _yr)) + _heading;
        if(angle >= (M_PI)) angle -= M_PI;
        if(angle <= -(M_PI)) angle += M_PI;

        _xr += odometryChange * sin(-_heading);
        _yr += odometryChange * cos(_heading);

        this->_radiusToGoal = sqrt( pow((_goalX - _xr),2) + pow((_goalY - _yr),2));
        this->_angleToGoal = angle;

        if(msg->direction == STOP)
        {
            // don't write data to file
        }
        else if(!((msg->direction == FORWARD) && !_rightOrLeftFlag))
        {
            _writer.WriteData(_sensors, _controls, (_radiusToGoal/_initialRadiusToGoal), _angleToGoal, (keypadDirection)msg->direction);
        }

        std::cout<<std::fixed<<std::setprecision(5)<<"radius: "<<_radiusToGoal<<", angle: "<<_angleToGoal*(180/M_PI);
        std::cout<<", "<<_sensors.GetLeftDistanceSensor().distance<<", "<<_sensors.GetCenterDistanceSensor().distance<<", "<<_sensors.GetRightDistanceSensor().distance<<std::endl;
    }
    else
    {
        std::cout<<"Finished Writing to File"<<std::endl;
    }
}

void
SystemInterface::publishData(lcm::LCM *lcm, SystemInterface *system, SensorManager *sensors)
{
    ANNLCM::neuralnetworkdata_t msg;
    msg.name = "NeuralNetworkData";

    while(system->_isRunning)
    {
        msg.enabled = true;

        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        msg.timestamp = (int64_t)ms.count();

        msg.left_sensor = sensors->GetLeftDistanceSensor().distance;
        msg.center_sensor = sensors->GetCenterDistanceSensor().distance;
        msg.right_sensor = sensors->GetRightDistanceSensor().distance;
        msg.radius_to_goal = system->_radiusToGoal;
        msg.angle_to_goal = system->_angleToGoal;

        lcm->publish(NeuralNetworkChannel, &msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    msg.enabled = false;
    lcm->publish(NeuralNetworkChannel, &msg);
}


