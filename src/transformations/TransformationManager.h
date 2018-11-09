#ifndef TRANSFORMATIONMANAGER_H
#define TRANSFORMATIONMANAGER_H

#include <ros/ros.h>
#include <string.h>
#include <map>

#include <boost/thread.hpp>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "../helper/TransformationHandler.h"
#include "../data/QRCodeData.h"

class TransformationManager
{
public:
    TransformationManager(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler);
    ~TransformationManager();

    void AddQrCodesData(std::vector<QRCodeData> qrCodes);

private:
    //node stuff
    ros::NodeHandle* _node;
    std::string _nodeName;
    void Run();
    boost::thread* _nodeThread;

    //parameter stuff
    customparameter::ParameterHandler* _parameterHandler;
    customparameter::Parameter<int> _paramRefreshRate;
    customparameter::Parameter<float> _paramPoseErrorFactor;

    void Init(ros::NodeHandle* parentNode);
    void InitParameter();
    void InitRosStuff();

    //transform stuff
    void PublishTransforms();
    helper::TransformationHandler* _transformationHandler;

    //qrcode stuff
    boost::mutex* _qrCodeMutex; //qrCode mutex for accessing qrCode elements
    std::vector<QRCodeData> _latestQrCodesData;
    std::vector<QRCodeData> GetQrCodes();
};

#endif // TRANSFORMATIONMANAGER_H
