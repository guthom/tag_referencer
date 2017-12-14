#include <ros/ros.h>
#include <string.h>

#include <ros/subscriber.h>
#include <qrcode_referencer/GetQRPose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

#include "src/data/QRCodeData.h"
#include "scanning/QRScanner.h"
#include "transformations/PoseDerivator.h"

//common stuff
std::string nodeName = "qrcode_referencer";

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subCameraInfo;
ros::Subscriber subImageMessage;
ros::Subscriber subDepthImageMessage;
ros::Publisher pubScannedImage;
ros::Publisher pubMarkedPointCloud;
ros::Publisher pubDebugPose;
ros::ServiceServer srvGetQRPose;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<std::string> paramCameraBaseTopic;
customparameter::Parameter<int> paramRefreshRate;
customparameter::Parameter<int> paramReferenceCorner;
customparameter::Parameter<bool> paramServiceMode;
customparameter::Parameter<bool> paramPublishMarkedImage;
customparameter::Parameter<bool> paramPublishMarkedPointCloud;

//scanning stuff
QRScanner _scanner;
PoseDerivator _poseDerivator;


//qrCode data
static boost::mutex _dataMutex;
std::vector<QRCodeData> _qrCodesData;
void SetQrCodesData(std::vector<QRCodeData> data)
{
    _dataMutex.lock();
    _qrCodesData = data;
    _dataMutex.unlock();
}

std::vector<QRCodeData> GetQrCodesData()
{
    _dataMutex.lock();
    std::vector<QRCodeData> qrCodesData = std::vector<QRCodeData>(_qrCodesData);
    _dataMutex.unlock();

    return qrCodesData;
}

static boost::mutex mutexImage;
//TODO: Use cvBridge only!
cv::Mat _cvImage;
cv::Mat GetCvImage()
{
    mutexImage.lock();
    cv::Mat image = cv::Mat(_cvImage);
    mutexImage.unlock();
    return image;
}

sensor_msgs::Image _currentImageMsg;
sensor_msgs::Image GetImageMsg()
{
    mutexImage.lock();
    sensor_msgs::Image image = sensor_msgs::Image(_currentImageMsg);
    mutexImage.unlock();
    return image;
}

static boost::mutex mutexDepth;
sensor_msgs::PointCloud2 _currentDepthMsg;
void depthCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    mutexDepth.lock();
    _currentDepthMsg = *msg;
    mutexDepth.unlock();
}
sensor_msgs::PointCloud2 GetDepthMsg()
{
    mutexDepth.lock();
    sensor_msgs::PointCloud2  depthMsg = sensor_msgs::PointCloud2(_currentDepthMsg);
    mutexDepth.unlock();
    return depthMsg;
}

static boost::mutex mutexCameraInfo;
sensor_msgs::CameraInfo _currentCameraInfo;
void cameraInfoCallback(const sensor_msgs::CameraInfo msg)
{
    mutexCameraInfo.lock();
    _currentCameraInfo = msg;
    mutexCameraInfo.unlock();
}

sensor_msgs::CameraInfo GetCameraInfo()
{
    mutexCameraInfo.lock();
    sensor_msgs::CameraInfo cameraInfo = sensor_msgs::CameraInfo(_currentCameraInfo);
    mutexCameraInfo.unlock();
    return cameraInfo;
}

void InitParams()
{
    //init params
    parameterHandler = new customparameter::ParameterHandler(node);
    std::string subNamespace = "";
    //Standard params
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)15);
    paramReferenceCorner = parameterHandler->AddParameter("ReferenceCorner", "", (int)1);
    paramServiceMode = parameterHandler->AddParameter("ServiceMode", "", false);
    paramPublishMarkedPointCloud = parameterHandler->AddParameter("PublishMarkedPointCloud", "", true);
    paramPublishMarkedImage = parameterHandler->AddParameter("PublishMarkedImage", "", true);
    paramCameraBaseTopic = parameterHandler->AddParameter("CameraBaseTopic", "", std::string("/zed/"));
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);

    mutexImage.lock();

    _cvImage = cv::Mat(cvImage->image);
    _currentImageMsg = *msg;

    mutexImage.unlock();
}

void PublishMarkedImage(cv::Mat image)
{
    using namespace cv_bridge;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    CvImage imageBridge = CvImage(header, sensor_msgs::image_encodings::RGB8, image);

    sensor_msgs::Image imgMsg;
    imageBridge.toImageMsg(imgMsg);
    pubScannedImage.publish(imgMsg);

}

void PublishDebugPose(std::vector<QRCodeData> qrCodes)
{
    geometry_msgs::PoseArray msg;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    msg.header = header;
    msg.header.frame_id = qrCodes[0].cameraFrameID;

    for(int i = 0; i < qrCodes.size(); i++)
    {
        msg.poses.push_back(qrCodes[i].qrPose);
    }
    pubDebugPose.publish(msg);
}

void MarkImage()
{
    cv::Mat markedImage = _scanner.MarkImage(GetQrCodesData(), paramReferenceCorner.GetValue(), GetCvImage());
    PublishMarkedImage(markedImage);
}

void ScanCurrentImg()
{
    auto currentImage = GetCvImage();
    auto qrCodeData = _scanner.ScanCurrentImg(currentImage);

    //set reference frame for all qrcodes
    auto frameID = GetImageMsg().header.frame_id;
    for(int i = 0; i < qrCodeData.size(); i++)
    {
        qrCodeData[i].cameraFrameID = frameID;
    }

    SetQrCodesData(qrCodeData);

}

void MarkPointCloud()
{
    //TODO: Implement this method
}

void FuseInformation()
{
    auto qrCodesData = GetQrCodesData();
    if(qrCodesData.size() <= 0)
        return;

    auto depthMsg = GetDepthMsg();
    pcl::PCLPointCloud2 pclCloud;
    pcl_conversions::moveToPCL(depthMsg, pclCloud);

    qrCodesData = _poseDerivator.CalculateQRPose(qrCodesData, pclCloud, paramReferenceCorner.GetValue());
    PublishDebugPose(qrCodesData);

    //publish MarkedPointCloud
    //TODO: Use MarkPointCloud instead
    if(paramPublishMarkedPointCloud.GetValue())
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        depthMsg.header = header;
        pubMarkedPointCloud.publish(depthMsg);
    }
}

bool GetQRPoseService(qrcode_referencer::GetQRPoseRequest &request, qrcode_referencer::GetQRPoseResponse &response)
{
    bool res = false;

    return res;
}

void Init()
{
    InitParams();
}

//used to save a lot of ifs every cycle
std::vector<void (*)()> processingFunctions;
int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    Init();

    //define subcriber
    subCameraInfo = node->subscribe(paramCameraBaseTopic.GetValue() + "depth/camera_info", 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "left/image_rect_color", 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "/point_cloud/cloud_registered", 1, depthCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    //define publisher
    pubScannedImage = node->advertise<sensor_msgs::Image>("ScannedImage", 100);
    ROS_INFO_STREAM("Will publish ScannedImages to " << pubScannedImage.getTopic());
    pubMarkedPointCloud = node->advertise<sensor_msgs::PointCloud2>("MarkedPointCloud", 100);
    ROS_INFO_STREAM("Will publish marked Pointclouds to " << pubMarkedPointCloud.getTopic());

    pubDebugPose = node->advertise<geometry_msgs::PoseArray>("DebugPose", 100);

    ros::Rate rate(paramRefreshRate.GetValue());

    if (paramServiceMode.GetValue())
    {
        srvGetQRPose = node->advertiseService("GetQRPose", GetQRPoseService);
        ROS_INFO("Ready to derive QRCode-Poses");

        ros::spin();
    }
    else
    {
        //define processing functions
        processingFunctions.push_back(ScanCurrentImg);

        if(paramPublishMarkedImage.GetValue())
        {
            processingFunctions.push_back(MarkImage);
        }

        processingFunctions.push_back(FuseInformation);

        if(paramPublishMarkedPointCloud.GetValue())
        {
            processingFunctions.push_back(MarkPointCloud);
        }

        while(node->ok())
        {
            ros::spinOnce();

            //continuously scann image
            if(_currentImageMsg.data.size() > 0)
            {
                for(int i = 0; i < processingFunctions.size(); i++)
                {
                    //execute added functions
                    (*processingFunctions[i])();
                }
            }

            rate.sleep();
        }
    }
    return 0;
}
