/*
 * ClassifyANN.cpp
 *
 *  Created on: 2019-03-21
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "ClassifyANN.hpp"

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

ClassifyANN::ClassifyANN()
{
    this->_blobShape = nullptr;
    this->_direction = STOP;
}

ClassifyANN::~ClassifyANN()
{

}

void
ClassifyANN::Initialize()
{
    _blobShape = _blobProto.mutable_shape();
    _blobShape->add_dim(1);
    _blobShape->add_dim(NumberInputs);
}

void
ClassifyANN::SetData(std::vector<float> inputData, caffe::Blob<float> &blob)
{
    const int dataSize = inputData.size();
    for(int i = 0; i < dataSize; ++i)
    {
        _blobProto.add_data((float)inputData[i]);
    }

    blob.FromProto(_blobProto);

    _bottom.clear(); //Clear existing data
    _bottom.push_back(&blob);
}

bool
ClassifyANN::Forward(caffe::Net<float> &net)
{
    if(_bottom.empty())
    {
        std::cout<<"[ERROR]: data is not set"<<std::endl;
        return false;
    }

    // forward pass
    float loss = 0.0;
    const std::vector<caffe::Blob<float>*>& result = net.Forward(_bottom, &loss);

    // find maximum
    float max = -1;
    int max_i = -1;
    for (int i = 0; i < NumberOutputs; ++i)
    {
        const float value = (result[0]->cpu_data())[i];
        std::cout << "index: " << i << " value: " << value << std::endl;
        if (value > max)
        {
            max = value;
            max_i = i;
        }
    }

    std::cout << "Result is: " << max_i << " classified"<<std::endl;
    _direction = (keypadDirection)max_i;

    return true;
}


