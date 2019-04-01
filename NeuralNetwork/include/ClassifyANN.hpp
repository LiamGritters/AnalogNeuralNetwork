/*
 * ClassifyANN.hpp
 *
 *  Created on: 2019-03-21
 *      Author: liam
 */

#ifndef NEURALNETWORK_INCLUDE_CLASSIFYANN_HPP_
#define NEURALNETWORK_INCLUDE_CLASSIFYANN_HPP_


/****************************************
 * INCLUDES
 ****************************************/

#include "RobotKeypadController.hpp"

#include <caffe/caffe.hpp>
#include <caffe/util/db.hpp>

#include <string>
#include <vector>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr int NumberInputs = 5; //left sensor, center sensor, right sensor, radius to goal, angle to goal
constexpr int NumberOutputs = 4; //STOP,FORWARD,LEFT,RIGHT

/****************************************
 * CLASS DEFINITION
 ****************************************/

class ClassifyANN
{

public:

    ClassifyANN();
    ~ClassifyANN();

    void Initialize();

    void SetData(std::vector<float> inputData, caffe::Blob<float> &blob);

    bool Forward(caffe::Net<float> &net); //Run instance with set data

    keypadDirection GetDirection(){return _direction;};

private:

    keypadDirection _direction;

    caffe::BlobProto _blobProto;
    caffe::BlobShape *_blobShape;

    std::vector<caffe::Blob<float>*> _bottom;
};



#endif /* NEURALNETWORK_INCLUDE_CLASSIFYANN_HPP_ */
