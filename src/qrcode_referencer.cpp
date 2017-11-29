#include <ros/ros.h>
#include <string.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include <boost/thread.hpp>

//common stuff
std::string nodeName = "qrcode_referencer";

//ros sutff
ros::NodeHandle* node;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<int> paramRefreshRate;

void InitParams()
{
    std::string subNamespace = "";
    //Standard params
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)20);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    //init params
    parameterHandler = new customparameter::ParameterHandler(node);
    InitParams();

    ros::Rate rate(paramRefreshRate.GetValue());
    while(node->ok())
    {        
        ros::spinOnce();
        rate.sleep();        
    }
    return 0;
}
