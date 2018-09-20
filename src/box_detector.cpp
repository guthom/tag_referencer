#include <ros/ros.h>
#include <string.h>

#include <ros/subscriber.h>
#include <qrcode_referencer/DetectBoxPoses.h>
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


//common stuff
std::string nodeName = "box_detector";

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subCameraInfo;
ros::Subscriber subImageMessage;
ros::Subscriber subDepthImageMessage;
ros::Publisher pubScannedImage;
ros::Publisher pubMarkedPointCloud;
ros::Publisher pubDebugPose;
ros::ServiceServer srvDetectBoxes;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<int> paramRefreshRate;
customparameter::Parameter<int> paramReferenceCorner;
customparameter::Parameter<bool> paramServiceMode;
customparameter::Parameter<bool> paramPublishMarkedImage;
customparameter::Parameter<bool> paramPublishMarkedPointCloud;


static boost::mutex mutexImage;
sensor_msgs::Image _currentImageMsg;
sensor_msgs::Image GetImageMsg()
{
    mutexImage.lock();
    sensor_msgs::Image image = sensor_msgs::Image(_currentImageMsg);
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
    paramPublishMarkedPointCloud = parameterHandler->AddParameter("PublishMarkedPointCloud", "", false);
    paramPublishMarkedImage = parameterHandler->AddParameter("PublishMarkedImage", "", true);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

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

void MarkImage()
{
    if(_currentImageMsg.data.size() > 0)
    {

    }
}

void ScanCurrentImg()
{
    cv::Mat currentImage = GetCvImage();
    if(!currentImage.empty())
    {

    }
}

void MarkPointCloud()
{
    //TODO: Implement this method
}


bool DetectBoxPosesService(qrcode_referencer::DetectBoxPosesRequest &request, qrcode_referencer::DetectBoxPosesResponse &response)
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
    subCameraInfo = node->subscribe("/depthcam1/rgb/camera_info", 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe("/depthcam1/color/image_raw", 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe("/depthcam1/depth_registered/points", 1, depthCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    //define publisher
    pubScannedImage = node->advertise<sensor_msgs::Image>("ScannedImage", 100);
    ROS_INFO_STREAM("Will publish ScannedImages to " << pubScannedImage.getTopic());
    pubMarkedPointCloud = node->advertise<sensor_msgs::PointCloud2>("MarkedPointCloud", 100);
    ROS_INFO_STREAM("Will publish marked Pointclouds to " << pubMarkedPointCloud.getTopic());

    ros::Rate rate(paramRefreshRate.GetValue());

    if (paramServiceMode.GetValue())
    {
        srvDetectBoxes = node->advertiseService("DetectBoxPoses", DetectBoxPosesService);
        ROS_INFO("Ready to derive Detect Box Poses");

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

        if(paramPublishMarkedPointCloud.GetValue())
        {
            processingFunctions.push_back(MarkPointCloud);
        }


        ROS_INFO_STREAM("Starting " + nodeName + " node");

        while(node->ok())
        {
            ros::spinOnce();

            //continuously scann image
            for(int i = 0; i < processingFunctions.size(); i++)
            {
                //execute added functions
                (*processingFunctions[i])();
            }

            rate.sleep();
        }
    }
    return 0;
}
