/*
 * NeuralNetworkController.cpp
 *
 *  Created on: 2019-03-21
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "ClassifyANN.hpp"

#include "keypadcontroller_t.hpp"
#include "neuralnetworkdata_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string NetworkChannel = "NEURAL";
const std::string ControllerChannel = "CONTROLLER";

const std::string CaffeNetwork = "./CaffeNetwork/deploy.prototxt";
const std::string CaffeWeights = "./CaffeNetwork/snapshot_iter_10000.caffemodel";

/****************************************
 * Main
 ****************************************/

class NeuralNetworkHandler
{
    public:
        NeuralNetworkHandler()
        {
            RobotDataMsgEnabled = false;
            RobotData.resize(5);
        }
        ~NeuralNetworkHandler() {}

        void
        HandleRobotDataMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const ANNLCM::neuralnetworkdata_t* msg)
        {
            RobotDataMsgEnabled = msg->enabled;

            RobotData[0] = msg->left_sensor;
            RobotData[1] = msg->center_sensor;
            RobotData[2] = msg->right_sensor;
            RobotData[3] = msg->radius_to_goal;
            RobotData[4] = msg->angle_to_goal;
        }

    public:
        std::vector<float> RobotData;
        bool RobotDataMsgEnabled;
};

int main()
{
    std::cout << "loading " << CaffeNetwork<< std::endl;
    caffe::Net<float> net(CaffeNetwork, caffe::TEST);
    std::cout << "loading " << CaffeWeights << std::endl;
    net.CopyTrainedLayersFrom(CaffeWeights);

    std::vector<int> vShape;
    vShape.push_back(1);
    vShape.push_back(NumberInputs);
    caffe::Blob<float> blob(vShape);

    ClassifyANN network;
    network.Initialize();

    lcm::LCM lcm;

    if(!lcm.good()) return -1;

    NeuralNetworkHandler handler;
    lcm.subscribe(NetworkChannel, &NeuralNetworkHandler::HandleRobotDataMessage, &handler);
    lcm.handle();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    ANNLCM::keypadcontroller_t msg;
    msg.enabled = true;
    msg.name = "controller";

    while(handler.RobotDataMsgEnabled)
    {
        lcm.handle();

        for(int i = 0; i < 5; ++i)
        {
            std::cout<<handler.RobotData[i]<<",";
        }
        std::cout<<std::endl;

        network.SetData(handler.RobotData, blob);

        if(!network.Forward(net)) return -1;

        msg.direction = network.GetDirection();
        lcm.publish(ControllerChannel, &msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    msg.enabled = false;
    lcm.publish(ControllerChannel, &msg);
}



