#include "TransformationManager.h"

TransformationManager::TransformationManager(ros::NodeHandle* parentNode, customparameter::ParameterHandler* parameterHandler)
    : _parameterHandler(parameterHandler)
{
    Init(parentNode);
}

void TransformationManager::Init(ros::NodeHandle* parentNode)
{
    _nodeName = "transformation_manager";
    _node = new ros::NodeHandle(*parentNode, _nodeName);

    //run node
    _transformationHandler = new helper::TransformationHandler(_node);
    _nodeThread = new boost::thread(boost::bind(&TransformationManager::Run,this));

}

void TransformationManager::InitParameter()
{
    std::string subNamespace = _nodeName + "/";
    _paramRefreshRate = _parameterHandler->AddParameter("RefreshRate", subNamespace, "", int(5));
    _paramPoseErrorFactor = _parameterHandler->AddParameter("PoseErrorFactor", subNamespace, "", 0.1f);
}

void TransformationManager::InitRosStuff()
{
}

void TransformationManager::AddQrCodesData(std::vector<QRCodeData> qrCodes)
{
    _qrCodeMutex->lock();
    _latestQrCodesData = qrCodes;
    _qrCodeMutex->unlock();
}


std::vector<QRCodeData> TransformationManager::GetQrCodes()
{
    _qrCodeMutex->lock();
    std::vector<QRCodeData> ret(_latestQrCodesData);
    _qrCodeMutex->unlock();

    return ret;
}

void TransformationManager::PublishTransforms()
{
    std::vector<QRCodeData> qrCodes = GetQrCodes();

    std_msgs::Header header;
    header.stamp = ros::Time::now();

    //create transforms for each entry
    for(int i = 0; i < qrCodes.size(); i++)
    {
        //create new transform and set header and frame ids
        geometry_msgs::TransformStamped transform;
        header.frame_id = qrCodes[i].cameraFrameID;
        transform.header = header;
        transform.child_frame_id = qrCodes[i].frameName;

        //set transfromation information
        geometry_msgs::Pose pose = qrCodes[i].qrPose;

        transform.transform.rotation = qrCodes[i].qrPose.orientation;
        transform.transform.translation.x = pose.position.x;
        transform.transform.translation.y = pose.position.y;
        transform.transform.translation.z = pose.position.z;


        if (std::isnan(transform.transform.rotation.w) || transform.transform.rotation.w == 0.0 ||
                std::isnan(transform.transform.translation.x))
        {
            //ROS_INFO_STREAM("qrcode_referencer: Got NAN value in Transformation!");
        }
        else
        {
            _transformationHandler->SendStableTransform(transform, _paramPoseErrorFactor.GetValue());
        }
    }
}

void TransformationManager::Run()
{
    InitParameter();

    //init publisher and marker
    InitRosStuff();

    //init mutex in  the same thread
    _qrCodeMutex = new boost::mutex();

    ROS_INFO_STREAM("Starting " + _nodeName + " node");

    ros::Rate rate(_paramRefreshRate.GetValue());
    while(_node->ok())
    {
        ros::spinOnce();

        PublishTransforms();

        rate.sleep();
    }
}

TransformationManager::~TransformationManager()
{

}
