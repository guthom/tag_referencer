#ifndef TRANSFORMATIONMANAGER_H
#define TRANSFORMATIONMANAGER_H

#include <ros/ros.h>
#include <string.h>
#include <map>

#include <boost/thread.hpp>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

class TransformationManager
{
public:
    TransformationManager(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler);

    ~TransformationManager();

private:
    //threadsafe stuff
    boost::mutex* _mutex; //main mutex for accessing shared elements

    //node stuff
    ros::NodeHandle* _node;
    std::string _nodeName;
    void Run();
    boost::thread* _nodeThread;

    //parameter stuff
    customparameter::ParameterHandler* _parameterHandler;
    customparameter::Parameter<int> _paramRefreshRate;

    void Init(ros::NodeHandle* parentNode);
    void InitParameter();
    void InitRosStuff();
};

#endif // TRANSFORMATIONMANAGER_H
