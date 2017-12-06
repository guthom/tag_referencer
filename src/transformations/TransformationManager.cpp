#include "TransformationManager.h"
#include <boost/thread.hpp>
TransformationManager::TransformationManager(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler)
    : _parameterHandler(parameterHandler)
{
    Init(parentNode);
}

void TransformationManager::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "solver_visualization";
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    //run node
    _nodeThread = new boost::thread(boost::bind(&TransformationManager::Run,this));
}

void TransformationManager::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _paramRefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(5));
}

void TransformationManager::InitRosStuff()
{
}

void TransformationManager::Run()
{
    InitParameter();

    //init publisher and marker
    InitRosStuff();

    //init mutex in  the same thread
    _mutex = new boost::mutex();
    ros::Rate rate(_paramRefreshRate.GetValue());
    while(_node->ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

TransformationManager::~TransformationManager()
{

}
