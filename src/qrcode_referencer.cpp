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

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subCameraInfo;
ros::Subscriber subImageMessage;
ros::Subscriber subDepthImageMessage;
ros::Publisher pubScannedImage;
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
static boost::mutex _dataMutex;
std::vector<image_data> _qrCodesData;
void setQrCodesData(std::vector<image_data> data)
{
    _dataMutex.lock();
    _qrCodesData = data;
    _dataMutex.unlock();
}

std::vector<image_data> GetQrCodesData()
{
    _dataMutex.lock();
    std::vector<image_data> qrCodesData = std::vector<image_data>(_qrCodesData);
    _dataMutex.unlock();

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

static boost::mutex mutexImage;
Image* _currentZbarImage = new Image();
Image GetZbarImage()
{
    mutexImage.lock();
    Image image = Image(*_currentZbarImage);
    mutexImage.unlock();
    return image;
}

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


void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);

    mutexImage.lock();

    _cvImage = cv::Mat(cvImage->image);

    cv::Mat grayImage;

    cv::cvtColor(cvImage->image, grayImage, CV_RGB2GRAY);

    //TODO: Check if memory leaks!
    _currentZbarImage = new Image((unsigned  int)grayImage.cols, (unsigned  int)grayImage.rows, "Y800",
                                  grayImage.data, (unsigned  int)(grayImage.cols * cvImage->image.rows));

    _currentImageMsg = *msg;

    mutexImage.unlock();
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

void MarkImage()
{
    using namespace cv;
    auto qrCodesData = GetQrCodesData();
    if (qrCodesData.size() <= 0)
    {
        return;
    }

    cv::Mat image = GetCvImage();
    for (int i = 0; i < qrCodesData.size(); i++)
    {
        Point center = Point(qrCodesData[i].pixX, qrCodesData[i].pixY);
        circle(image, center, 5 , Scalar( 255, 0, 0), 4);

        center.x += 10;
        center.y -= 5;
        //set text with frame name
        putText(image, qrCodesData[i].frameName ,center, FONT_HERSHEY_SIMPLEX, 1, Scalar( 255, 0, 0) ,2, LINE_AA);
    }

    PublishMarkedImage(image);
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
    auto qrCodes = GetQrCodesData();
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

void Init()
{
    InitParams();
    InitZBar();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    Init();

    //define subcriber
    subCameraInfo = node->subscribe(paramCameraBaseTopic.GetValue() + "depth/camera_info", 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "right/image_rect_color", 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe(paramCameraBaseTopic.GetValue() + "depth/depth_registered", 1, depthCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    pubScannedImage = node->advertise<sensor_msgs::Image>("ScannedImage", 100);
    ROS_INFO_STREAM("Will publish ScannedImages to " << pubScannedImage.getTopic());

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
                MarkImage();
            }

            rate.sleep();
        }
    }
    return 0;
}
