#include <ros/ros.h>
#include <string.h>

#include <ros/subscriber.h>
#include <qrcode_referencer/GetQRPose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "zbar.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

struct image_data{
    bool success = false;
    int id;
    std::string info = "";
    std::string frameName = "";
    int pixX = 0;
    int pixY = 0;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;
};

//common stuff
std::string nodeName = "qrcode_referencer";

//boost stuff
boost::mutex* _mutex;

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subCameraInfo;
ros::Subscriber subImageMessage;
ros::Subscriber subDepthImageMessage;
ros::ServiceServer srvGetQRPose;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<std::string> paramCameraBaseTopic;
customparameter::Parameter<int> paramRefreshRate;
customparameter::Parameter<bool> paramServiceMode;

//zbar stuff
using namespace zbar;
ImageScanner imgScanner;

//qrCode data
std::vector<image_data>* _qrCodesData;
void setQrCodesData(std::vector<image_data> data)
{
    _mutex->lock();
    _qrCodesData = &data;
    _mutex->unlock();
}

std::vector<image_data> getQrCodesData()
{
    _mutex->lock();
    std::vector<image_data> qrCodesData = std::vector<image_data>(*_qrCodesData);
    _mutex->unlock();

    return qrCodesData;
}


void InitParams()
{
    //init params
    parameterHandler = new customparameter::ParameterHandler(node);
    std::string subNamespace = "";
    //Standard params
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)1);
    paramServiceMode = parameterHandler->AddParameter("ServiceMode", "", false);
    paramCameraBaseTopic = parameterHandler->AddParameter("CameraBaseTopic", "", std::string("/zed/"));
}

void InitZBar()
{
    //TODO: maybe theres a better parameterset for the configuration
    imgScanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
}

void Init()
{
    _mutex = new boost::mutex();
    InitParams();
    InitZBar();
}

void ProcessRawString(std::string rawString, image_data* retData)
{
    std::vector<std::string> qrCode;
    boost::split(qrCode, rawString, boost::is_any_of(",;[]"));

    try
    {
        retData->id = std::stoi(qrCode[0]);
        retData->success = true;
        retData->info=qrCode[1];
        retData->frameName=qrCode[2];
        //parse framePose given within the qrcode information
        retData->framePose.position.x = std::stof(qrCode[3]);
        retData->framePose.position.y = std::stof(qrCode[4]);
        retData->framePose.position.z = std::stof(qrCode[5]);
        retData->framePose.orientation.x = std::stof(qrCode[6]);
        retData->framePose.orientation.y = std::stof(qrCode[7]);
        retData->framePose.orientation.z = std::stof(qrCode[8]);
        retData->framePose.orientation.w = std::stof(qrCode[9]);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while parsing QR-Code Information, wrong format???");
    }
}

boost::mutex mutexImage;
Image* _currentZbarImage;
Image GetZbarImage()
{
    mutexImage.lock();
    Image image = Image(*_currentZbarImage);
    mutexImage.unlock();
    return image;
}

sensor_msgs::Image currentImageMsg;
void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg, "mono8");

    mutexImage.lock();
    //TODO: Check if memory leaks!
    _currentZbarImage = new Image((unsigned  int)cv_image->image.cols, (unsigned  int)cv_image->image.rows, "Y800",
                     cv_image->image.data, (unsigned  int)(cv_image->image.cols * cv_image->image.rows));

    currentImageMsg = *msg;
    mutexImage.unlock();
}

boost::mutex mutexDepth;
sensor_msgs::PointCloud2 _currentDepthMsg;
void deptCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    mutexDepth.lock();
    _currentDepthMsg = *msg;
    mutexDepth.unlock();
}

sensor_msgs::PointCloud2 getDepthMsg()
{
    mutexDepth.lock();
    sensor_msgs::PointCloud2  depthMsg = sensor_msgs::PointCloud2(_currentDepthMsg);
    mutexDepth.unlock();
    return depthMsg;
}

boost::mutex mutexCameraInfo;
sensor_msgs::CameraInfo _currentCameraInfo;
void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    mutexCameraInfo.lock();
    _currentCameraInfo = *msg;
    mutexCameraInfo.unlock();
}

sensor_msgs::CameraInfo GetCameraInfo()
{
    mutexCameraInfo.lock();
    sensor_msgs::CameraInfo cameraInfo = sensor_msgs::CameraInfo(_currentCameraInfo);
    mutexCameraInfo.unlock();
    return cameraInfo;
}

bool isProcessing = false;
void ScanCurrentImg()
{
    isProcessing = true;
    std::vector<image_data> qrCodes;

    Image image = GetZbarImage();

    int res = imgScanner.scan(image);

    if(res == -1)
    {
        ROS_INFO_STREAM("Error occurred while scanning image!");
    }
    else if (res >= 0)
    {
        //extract symbol information
        int counter = 0;
        for(Image::SymbolIterator symbol = image.symbol_begin(); symbol !=  image.symbol_end(); ++symbol)
        {
            std::string test = symbol->get_data();

            image_data entry;
            ProcessRawString(symbol->get_data().c_str(), &entry);

            entry.pixX = symbol->get_location_x(counter);
            entry.pixY = symbol->get_location_y(counter);

            qrCodes.push_back(entry);
            counter += 1;
        }

        //print res TODO: drop after debugphase
        ROS_INFO_STREAM("Decoded " << res << " QRCodes");
        for(int i = 0; i < qrCodes.size(); i++)
        {
            ROS_INFO_STREAM(qrCodes[i].frameName + "; "<< qrCodes[i].pixY << "; " << qrCodes[i].pixX);
        }

        setQrCodesData(qrCodes);
    }
    isProcessing = false;
}

void FuseInformation()
{
    auto qrCodes = getQrCodesData();
    //sensor_msgs::PointCloud2 depthMsg = getCurrentDepthMsg();
    if(qrCodes.size() > 0)
    {
    }

}

bool GetQRPoseService(qrcode_referencer::GetQRPoseRequest &request, qrcode_referencer::GetQRPoseResponse &response)
{
    bool res = false;




    return res;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    Init();
    subCameraInfo = node->subscribe(paramCameraBaseTopic.GetValue() + "depth/camera_info", 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "right/image_rect_color", 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "depth/depth_registered", 1, deptCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    ros::Rate rate(paramRefreshRate.GetValue());

    if (paramServiceMode.GetValue())
    {
        srvGetQRPose = node->advertiseService("GetQRPose", GetQRPoseService);
        ROS_INFO("Ready to derive QRCode-Poses");

        ros::spin();
    }
    else
    {
        while(node->ok())
        {
            ros::spinOnce();

            //continously scann image
            if(_currentZbarImage)
            {
                ScanCurrentImg();
                FuseInformation();
            }

            rate.sleep();
        }
    }
    return 0;
}
