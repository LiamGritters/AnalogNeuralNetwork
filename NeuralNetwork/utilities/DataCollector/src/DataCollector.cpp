/*
 * DataCollector.cpp
 *
 *  Created on: 2019-04-07
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "DataCollector.hpp"

#include <cstdlib>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * NAMESPACES
 ****************************************/

using namespace ANN;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

DataCollector::DataCollector()
{
    this->_direction = STOP;
    this->_quitFlag = false;

    this->_radius = 0.0;
    this->_angle = 0.0;

    this->_rightSensor = 0.0;
    this->_leftSensor = 0.0;
    this->_centerSensor = 0.0;
}

DataCollector::~DataCollector()
{
    CloseFile();
}

void
DataCollector::GenerateSituation()
{
    generateData();
    displayData();
    getDirectionInput();
}

void
DataCollector::generateData()
{
    _rightSensor = (float)(rand() % 20000 ) / 100.f; //generate decimal number in the range 0 to 100
    _leftSensor = (float)(rand() % 20000 ) / 100.f;
    _centerSensor = (float)(rand() % 20000 ) / 100.f;

    if(_rightSensor > 100) _rightSensor = 100;
    if(_leftSensor > 100) _leftSensor = 100;
    if(_centerSensor > 100) _centerSensor = 100;

    _radius = (float)(rand() % 4000 ) / 100.f; //generate decimal number in the range of 0 to 10

    if(_radius > 10) _radius = 10.0;

    _angle = (((float)(rand() % 9000 ) / 100.f) - 45.0) * (M_PI / 180.f); //generate decimal number in the range of -90 to 90
}

void
DataCollector::displayData()
{
    cv::namedWindow( "Display window", CV_WINDOW_NORMAL );// Create a window for display.

    cv::Mat image(600, 400, CV_8U, cv::Scalar(0));

    const float leftx = 100 + _leftSensor*5.0*sin(-5.0 * M_PI/180.0);
    const float lefty = 600 - _leftSensor*5.0*cos(-5.0 * M_PI/180.0);
    const float rightx = 300 + _rightSensor*5.0*sin(5.0 * M_PI/180.0);
    const float righty = 600 - _rightSensor*5.0*cos(5.0 * M_PI/180.0);
    const float center = 600 - _centerSensor*5.0;

    cv::line(image, cv::Point(100,600), cv::Point(leftx,lefty),cv::Scalar(255), 3, 8, 0);
    cv::line(image, cv::Point(300,600), cv::Point(rightx,righty),cv::Scalar(255), 3, 8, 0);
    cv::line(image, cv::Point(200,600), cv::Point(200,center),cv::Scalar(255), 2, 3, 0);

    const float destx = 200*sin(-_angle) + 200;
    const float desty = (_radius*600*cos(_angle) > 600) ? 0 : 600 - _radius*600*cos(_angle);
    cv::circle(image, cv::Point(destx,desty),10, cv::Scalar(125), 10, 8, 0);

    cv::imshow( "Display window", image );
}

void
DataCollector::getDirectionInput()
{
    char c = cv::waitKey(0);

    switch((int)c)
    {   case 82: //Up arrow key
            _direction = FORWARD;
            break;
        case 81: //Left arrow key
            _direction = LEFT;
            break;
        case 83: //Right arrow key
            _direction = RIGHT;
            break;
        case 115: //s or stop key
            _direction = STOP;
            break;
        case 113: //q or quit key
            _quitFlag = true;
            break;
        default:
            break;
    }
}

void
DataCollector::WriteData(float leftSensor, float centerSensor, float rightSensor, float radiusToGoal, float angleToGoal, keypadDirection direction)
{
    if(_fileWriter.is_open())
    {
        _fileWriter << 1.0 - (leftSensor / 100.0) <<",";
        _fileWriter << 1.0 - (centerSensor / 100.0) <<",";
        _fileWriter << 1.0 - (rightSensor / 100.0) <<",";
        _fileWriter << radiusToGoal / 10.0 <<","; //right in terms of percent to target, with 10.0 being the max radius
        _fileWriter << (angleToGoal / M_PI) <<",";
        _fileWriter << direction <<std::endl;
    }
}

void
DataCollector::Initialize(std::string filename)
{
    this->_fileWriter.open(filename, std::ofstream::out | std::ofstream::app);
}

void
DataCollector::CloseFile()
{
    if(_fileWriter.is_open())
    {
        this->_fileWriter.close();
    }
}


