#include <ros/ros.h>
#include <string.h>

#include <vector>
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
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include <boost/thread.hpp>
#include <geometry_msgs/PoseStamped.h>

#include "src/data/QRCodeData.h"
#include "scanning/ScannerBase.h"
#include "scanning/QRScanner.h"
#include "scanning/AprilTagScanner.h"
#include "transformations/PoseDerivator.h"
#include "transformations/TransformationManager.h"

//common stuff
std::string nodeName = "april_calibrator";

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
customparameter::Parameter<int> paramRefreshRate;
customparameter::Parameter<int> paramReferenceCorner;
customparameter::Parameter<int> paramCalibCount;
customparameter::Parameter<int> paramCalibTagID;
customparameter::Parameter<bool> paramServiceMode;
customparameter::Parameter<bool> paramPublishMarkedImage;
customparameter::Parameter<bool> paramPublishMarkedPointCloud;
customparameter::Parameter<bool> paramSimulationMode;
customparameter::Parameter<bool> paramAprilTagMode;
customparameter::Parameter<bool> paramQRCodeMode;
customparameter::Parameter<float> paramMinPointDistance;

//scanning stuff
std::vector<ScannerBase*> _scanner;
PoseDerivator _poseDerivator;

//Transformation manager
TransformationManager* _transformManager;

//qrCode data
static boost::mutex _dataMutex;
std::vector<QRCodeData> _calibTargets;

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
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)1);
    paramReferenceCorner = parameterHandler->AddParameter("ReferenceCorner", "", (int)0);
    paramCalibCount = parameterHandler->AddParameter("CalibCount", "", (int)10);
    paramCalibTagID = parameterHandler->AddParameter("CalibTargetName", "", 0);
    paramMinPointDistance = parameterHandler->AddParameter("MinPointDistance", "", 0.01f);
    paramServiceMode = parameterHandler->AddParameter("ServiceMode", "", false);
    paramPublishMarkedPointCloud = parameterHandler->AddParameter("PublishMarkedPointCloud", "", false);
    paramPublishMarkedImage = parameterHandler->AddParameter("PublishMarkedImage", "", true);
    paramSimulationMode = parameterHandler->AddParameter("SimulationMode", "", false);
    paramQRCodeMode = parameterHandler->AddParameter("QRCodeMode", "", false);
    paramAprilTagMode = parameterHandler->AddParameter("AprilTagMode", "", true);
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

void PublishDebugPose(std::vector<QRCodeData> qrCodes)
{
    geometry_msgs::PoseArray msg;
    std_msgs::Header header;
    //header.stamp = ros::Time::now();
    msg.header = header;
    msg.header.frame_id = qrCodes[0].cameraFrameID;

    for(int i = 0; i < qrCodes.size(); i++)
    {
        msg.poses.push_back(qrCodes[i].qrPose);

        /*
        for (int j = 0; j < qrCodes[i].points3D.size(); j++) {
            geometry_msgs::Pose pose;
            pose.position.x = qrCodes[i].points3D[j][0];
            pose.position.y = qrCodes[i].points3D[j][1];
            pose.position.z = qrCodes[i].points3D[j][2];

            msg.poses.push_back(pose);
        }
        */
    }
        pubDebugPose.publish(msg);
}

void MarkImage()
{
    if(_currentImageMsg.data.size() > 0)
    {
        cv::Mat markedImage = GetCvImage();

        ScannerBase::MarkImage(GetQrCodesData(), paramReferenceCorner.GetValue(), markedImage);

        PublishMarkedImage(markedImage);
    }
}

void ScanCurrentImg()
{
    cv::Mat currentImage = GetCvImage();
    if(!currentImage.empty()) {
        std::vector<QRCodeData> qrCodeData;

        for (int i = 0; i < _scanner.size(); i++)
        {
            std::vector<QRCodeData> newQRCodeData = _scanner[i]->ScanCurrentImg(currentImage);

            //reload image and scan again if first scan fails
            if (newQRCodeData.size() == 0)
            {
                currentImage = GetCvImage();
                newQRCodeData = _scanner[i]->ScanCurrentImg(currentImage);
            }

            if (newQRCodeData.size() > 0)
            {
                qrCodeData.reserve(qrCodeData.size() + newQRCodeData.size());
                qrCodeData.insert(std::end(qrCodeData), std::begin(newQRCodeData), std::end(newQRCodeData));
            }
        }

        //set reference frame for all qrcodes
        auto frameID = GetImageMsg().header.frame_id;
        for (int i = 0; i < qrCodeData.size(); i++) {
            qrCodeData[i].cameraFrameID = frameID;
        }

        SetQrCodesData(qrCodeData);
    } else{
        ROS_WARN_STREAM("QRCodeReferencer: Got no image information!");
    }

}

void MarkPointCloud()
{
    //TODO: Implement this method
}

void PrintMatrix(MatrixXd mat)
{
    Eigen::IOFormat format;
    if (mat.cols() > 1)
        format = Eigen::IOFormat(StreamPrecision, 0, ", ", ";\n", "", "", "[\n ", "]");
    else
        format = Eigen::IOFormat(StreamPrecision, 0, ", ", ";  ", "", "", "[", "]");

    ROS_INFO_STREAM(mat.format(format));
}


void PrintMatrix(cv::Mat mat)
{
    ROS_INFO_STREAM(mat.rowRange(0, 3));
}

bool CheckDistance(cv::Point3d point1, cv::Point3d point2)
{
    float minDistance = paramMinPointDistance.GetValue();

    double res = cv::norm(point1-point2);

    if (res >= minDistance)
        return true;
    else
        return false;
}

void Calibrate()
{
    using namespace cv;
    using namespace std;

    // 2D/3D points
    vector<Point2d> imagePoints;
    vector<Point3d> modelPoints;

    for(int i = 0; i < _calibTargets.size(); i++)
    {
        auto target = _calibTargets[i];

        for(int j = 0; j < target.points.size(); j++)
        {
            imagePoints.push_back(Point2d(double(target.points[j][0]),
                                           double(target.points[j][1])));

            ROS_INFO_STREAM(to_string(imagePoints[imagePoints.size()-1].x) + ", " +
                                    to_string(imagePoints[imagePoints.size()-1].y));

            modelPoints.push_back(Point3d(double(target.points3D[j][0]),
                                           double(target.points3D[j][1]),
                                           double(target.points3D[j][2])));

            ROS_INFO_STREAM(to_string(modelPoints[modelPoints.size()-1].x) + ", " +
                            to_string(modelPoints[modelPoints.size()-1].y)+ ", " +
                            to_string(modelPoints[modelPoints.size()-1].z));
        }
    }

    if(imagePoints.size() != modelPoints.size())
    {
        ROS_INFO_STREAM("Can't run calibration! model and image point size is not equal!");
        return;
    }


    ROS_INFO_STREAM("Run calibration with " + to_string(imagePoints.size()) + " correspondences!");

    Mat intMat(3, 3, CV_64FC1, (void *) _currentCameraInfo.K.data());
    Mat distCoeffs(4, 1, CV_64FC1, (void *) _currentCameraInfo.D.data());

    PrintMatrix(intMat);
    PrintMatrix(distCoeffs);

    Mat rVec;
    Mat tVec;

    solvePnP(modelPoints, imagePoints, intMat, Mat::zeros(4, 1, CV_64FC1), rVec, tVec, false, CV_EPNP);

    ROS_INFO_STREAM(to_string(tVec.at<double>(0)) + ", " + to_string(tVec.at<double>(1)) + ", "
                    + to_string(tVec.at<double>(2)));

    //now we need the inverse matrix of the calculated transformation
    Mat rot, trans, jac;
    Rodrigues(rVec, rot, jac);

    //inverse of rotation = transpose!
    rot = rot.t();

    //inverse of translation
    trans = -rot * tVec;

    ROS_INFO_STREAM("Calibration result");
    ROS_INFO_STREAM(to_string(trans.at<double>(0)) + ", " + to_string(trans.at<double>(1)) + ", "
                    + to_string(trans.at<double>(2)));


    Eigen::Matrix3d mat;
    cv2eigen(rot, mat);

    Eigen::Quaternion<double> quat(mat);

    ROS_INFO_STREAM(to_string(quat.x()) + ", " + to_string(quat.y()) + ", " + to_string(quat.z()) + ", "
                    + to_string(quat.w()));
}

cv::Point3d lastValidPoint(0.0, 0.0, 0.0);
void CheckCalibrationTargets()
{
    auto newCodes = GetQrCodesData();
    unsigned long collectionSize = _calibTargets.size();
    if(collectionSize >= paramCalibCount.GetValue())
    {
        Calibrate();
        _calibTargets.clear();

        ROS_INFO_STREAM("Reset calibration collection!");
    }
    else if(newCodes.size() > 0)
    {
        int id = paramCalibTagID.GetValue();
        for(int i = 0; i < newCodes.size(); i++ )
        {
            if(newCodes[i].id == id)
            {
                Point3d point(newCodes[i].points3D[0][0], newCodes[i].points3D[0][1], newCodes[i].points3D[0][2]);

                if(CheckDistance(lastValidPoint, point))
                {
                    lastValidPoint = point;
                    _calibTargets.push_back(newCodes[i]);
                    ROS_INFO_STREAM("Add new calib target to calibration collection, Count:" +
                                    std::to_string(collectionSize));
                }
                else
                {
                    ROS_INFO_STREAM("Rejected point because its to close to the last one!");
                }
            }
        }

    }
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
    //send derived Transforms to transformation manager
    _transformManager->AddQrCodesData(qrCodesData);
    SetQrCodesData(qrCodesData);

    //publish MarkedPointCloud
    //TODO: Use MarkPointCloud instead
    if(paramPublishMarkedPointCloud.GetValue())
    {
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        depthMsg.header = header;
        pubMarkedPointCloud.publish(depthMsg);
        ROS_INFO_STREAM("Marked point cloud is not implemented yet.... I'm so sorry :-)");
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

    if (paramQRCodeMode.GetValue())
    {
        _scanner.push_back(new QRScanner(parameterHandler));
    };

    if (paramAprilTagMode.GetValue())
    {
        _scanner.push_back(new AprilTagScanner(parameterHandler));
    };

}

void PublishSimulatedQR()
{
    auto qrCodeData = GetQrCodesData();

    QRCodeData simulatedData;
    simulatedData.id = 100;
    simulatedData.cameraFrameID = "world";
    simulatedData.frameName ="simulatedQR0";
    simulatedData.qrPose.position.x = 0.8;
    simulatedData.qrPose.position.y = 0.3;
    simulatedData.qrPose.position.z = 0.8;

    simulatedData.qrPose.orientation.x = 0.478023;
    simulatedData.qrPose.orientation.y = -0.5422;
    simulatedData.qrPose.orientation.z = 0.573638;
    simulatedData.qrPose.orientation.w = -0.385267;


    //append simualted Pose information
    qrCodeData.push_back(simulatedData);

    simulatedData.qrPose.position.y = -0.2;

    simulatedData.qrPose.orientation.x =  0.751341;
    simulatedData.qrPose.orientation.y = 0.0315404;
    simulatedData.qrPose.orientation.z = 0.65891;
    simulatedData.qrPose.orientation.w = 0.0181433;

    simulatedData.frameName ="simulatedQR1";
    qrCodeData.push_back(simulatedData);

    simulatedData.qrPose.position.y = 0.0;
    simulatedData.frameName ="simulatedQR2";
    qrCodeData.push_back(simulatedData);

    simulatedData.qrPose.position.z = 0.3;
    simulatedData.qrPose.orientation.x = 0.478023;
    simulatedData.qrPose.orientation.y = -0.5422;
    simulatedData.qrPose.orientation.z = 0.573638;
    simulatedData.qrPose.orientation.w = -0.385267;
    simulatedData.frameName ="simulatedQR3";
    qrCodeData.push_back(simulatedData);

    simulatedData.qrPose.position.z = 0.5;
    simulatedData.qrPose.position.x = 1.0;
    simulatedData.qrPose.position.y = 0.0;

    simulatedData.qrPose.orientation.x =  0.751341;
    simulatedData.qrPose.orientation.y = 0.0315404;
    simulatedData.qrPose.orientation.z = 0.65891;
    simulatedData.qrPose.orientation.w = 0.0181433;
    simulatedData.frameName ="simulatedQR4";
    qrCodeData.push_back(simulatedData);

    _transformManager->AddQrCodesData(qrCodeData);

}

//used to save a lot of ifs every cycle
std::vector<void (*)()> processingFunctions;
int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    Init();

    //define subcriber
    subCameraInfo = node->subscribe(node->resolveName("/depthcam1/depthcam1/color/camera_info"), 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe(node->resolveName("/depthcam1/depthcam1/color/image_raw"), 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe(node->resolveName("/depthcam1/depthcam1/depth_registered/points"), 1, depthCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    //define publisher
    pubScannedImage = node->advertise<sensor_msgs::Image>("ScannedImage", 100);
    ROS_INFO_STREAM("Will publish ScannedImages to " << pubScannedImage.getTopic());
    pubMarkedPointCloud = node->advertise<sensor_msgs::PointCloud2>("MarkedPointCloud", 100);
    ROS_INFO_STREAM("Will publish marked Pointclouds to " << pubMarkedPointCloud.getTopic());

    pubDebugPose = node->advertise<geometry_msgs::PoseArray>("DebugPose", 100);

    //launch transformation manager
    _transformManager = new TransformationManager(node, parameterHandler);

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

        if(paramSimulationMode.GetValue())
        {
            ROS_WARN_STREAM("QRCodeReferencer will Publish simulated QRCode pose!");
            processingFunctions.push_back(PublishSimulatedQR);
        }

        processingFunctions.push_back(CheckCalibrationTargets);

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
