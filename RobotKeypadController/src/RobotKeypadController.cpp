/*
 * RobotKeypadController.cpp
 *
 *  Created on: 2019-01-10
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "RobotKeypadController.hpp"

#include <iostream>
#include <ncurses.h>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

RobotKeypadController::RobotKeypadController()
{
    this->_isRunning = false;
    this->_direction = STOP;
    this->_radius = 0.0;
    this->_angle = 0.0;
}

RobotKeypadController::~RobotKeypadController()
{
    Stop();
}

void
RobotKeypadController::Start()
{
    this->_isRunning = true;
    this->_thread = std::thread(RobotKeypadController::controllerThread, this);
}

void
RobotKeypadController::Stop()
{
    this->_isRunning = false;
    if(_thread.joinable())
    {
        _thread.join();
    }
}

void
RobotKeypadController::controllerThread(RobotKeypadController *controller)
{
    initscr();
    int c;

    raw();
    keypad(stdscr, TRUE);
    noecho();

    while(controller->_isRunning)
    {
        c = 's';
        c = getch();
        timeout(100);
        switch(c)
        {   case KEY_UP:
                controller->_direction = FORWARD;
                break;
            case KEY_LEFT:
                controller->_direction = LEFT;
                break;
            case KEY_RIGHT:
                controller->_direction = RIGHT;
                break;
            default:
                controller->_direction = STOP;
                refresh();
                break;
        }

        if((c == 'q') || (c == 27))
        {
            printw("Stopping Controller Thread \n");
            break;
        }
        refresh();
    }
    controller->_isRunning = false;

    refresh();
    endwin();
}


