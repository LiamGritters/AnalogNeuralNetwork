/*
 * SystemInterface.hpp
 *
 *  Created on: 2019-01-08
 *      Author: liam
 */

#ifndef SYSTEMINTERFACE_HPP_
#define SYSTEMINTERFACE_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "SensorManager.hpp"
#include "ControlsManager.hpp"
#include "RobotDataWriter.hpp"

#include "keypadcontroller_t.hpp"

#include <lcm/lcm-cpp.hpp>

#include <webots/Robot.hpp>
#include <mutex>
#include <thread>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class SystemInterface
{

public:

    SystemInterface();
    ~SystemInterface();

    bool Initialize(float radius, float angle); //location of goal from robot
    void Start();
    void Stop();
    void WriteData();

private:

    static void robotProcess(SystemInterface *sysInterface);

    static void publishData(lcm::LCM *lcm, SystemInterface *system, SensorManager *sensors);

    bool setupNetwork();

    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const ANNLCM::keypadcontroller_t *msg);

private:

    bool _isRunning;

    int _timeStep;
    int _webotsFlag;

    float _rightSpeed;
    float _leftSpeed;

    webots::Robot* _robot;

    std::thread _thread;
    std::thread _publishThread;

    SensorManager _sensors;
   ControlsManager _controls;

   RobotDataWriter _writer;

   float _initialRadiusToGoal;
   float _radiusToGoal;
   float _angleToGoal;
   float _odometry;
   float _heading;

   float _goalX;
   float _goalY;

   float _xr; //Robot Position
   float _yr;

   bool _rightOrLeftFlag;

    lcm::LCM _lcm;
};



#endif /* SYSTEMINTERFACE_HPP_ */
