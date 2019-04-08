/*
 * DataCollector.hpp
 *
 *  Created on: 2019-04-07
 *      Author: liam
 */

#ifndef NEURALNETWORK_UTILITIES_DATACOLLECTOR_INCLUDE_DATACOLLECTOR_HPP_
#define NEURALNETWORK_UTILITIES_DATACOLLECTOR_INCLUDE_DATACOLLECTOR_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "RobotKeypadController.hpp"

#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/****************************************
 * CLASS DEFINITION
 ****************************************/

namespace ANN
{

class DataCollector
{

public:

    DataCollector();
    ~DataCollector();

    void GenerateSituation(); //randomly generates analog neural network robot situation

    void WriteData(float leftSensor, float centerSensor, float rightSensor, float radiusToGoal, float angleToGoal, keypadDirection direction);
    void Initialize(std::string filename);
    void CloseFile();

    inline keypadDirection GetDirection() const {return _direction;};
    inline bool GetQuitFlag() const {return _quitFlag;};
    inline float GetRightSensor() const {return _rightSensor;};
    inline float GetCenterSensor() const {return _centerSensor;};
    inline float GetLeftSensor() const {return _leftSensor;};
    inline float GetRadius() const {return _radius;};
    inline float GetAngle() const {return _angle;};

private:

    void generateData();
    void displayData();
    void getDirectionInput();

private:

    keypadDirection _direction;

    float _radius;
    float _angle;

    float _rightSensor;
    float _leftSensor;
    float _centerSensor;

    bool _quitFlag;

    std::ofstream _fileWriter;
};

}

#endif /* NEURALNETWORK_UTILITIES_DATACOLLECTOR_INCLUDE_DATACOLLECTOR_HPP_ */
