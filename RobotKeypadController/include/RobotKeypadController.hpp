/*
 * RobotKeypadController.hpp
 *
 *  Created on: 2019-01-09
 *      Author: liam
 */

#ifndef ROBOTKEYPADCONTROLLER_HPP_
#define ROBOTKEYPADCONTROLLER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <thread>

/****************************************
 * ENUMS
 ****************************************/

enum keypadDirection
{
    STOP = 0,
    FORWARD = 1,
    LEFT = 2,
    RIGHT = 3
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class RobotKeypadController
{

public:

    RobotKeypadController();
    ~RobotKeypadController();

    void Start();

    void Stop();

    inline bool IsRunning() const {return _isRunning;};

    keypadDirection GetDirection(){return _direction;};

private:

    bool setupNetwork();

    static void controllerThread(RobotKeypadController *controller);

private:

    bool _isRunning;

    std::thread _thread;

    keypadDirection _direction;

    float _radius;
    float _angle;
};



#endif /* ROBOTKEYPADCONTROLLER_HPP_ */
