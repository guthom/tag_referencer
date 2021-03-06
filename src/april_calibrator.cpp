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
#include "helper/TransformationHandler.h"

//common stuff
std::string nodeName = "april_calibrator";

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subCameraInfo;
ros::Subscriber subImageMessage;
ros::Subscriber subDepthImageMessage;
ros::Publisher pubScannedImage;
ros::Publisher pubReporjectedImage;
ros::Publisher pubMarkedPointCloud;
ros::Publisher pubDebugPose;
ros::ServiceServer srvGetQRPose;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<int> paramRefreshRate;
customparameter::Parameter<int> paramReferenceCorner;
customparameter::Parameter<int> paramCalibCount;
customparameter::Parameter<int> paramCalibTagID;
customparameter::Parameter<std::string> paramCalibTargetTfName;
customparameter::Parameter<std::string> paramCameraName;
customparameter::Parameter<float> paramTagSize;
customparameter::Parameter<bool> paramServiceMode;
customparameter::Parameter<bool> paramPublishMarkedImage;
customparameter::Parameter<bool> paramPublishMarkedPointCloud;
customparameter::Parameter<bool> paramSimulationMode;
customparameter::Parameter<bool> paramAprilTagMode;
customparameter::Parameter<bool> paramQRCodeMode;
customparameter::Parameter<float> paramMinPointDistance;

//scanning stuff
std::vector<ScannerBase*> _scanner;
PoseDerivator* _poseDerivator;

//Transformation manager
TransformationManager* _transformManager;
helper::TransformationHandler* _transformHandler;

geometry_msgs::Pose _lastCalibPose;

std::vector<std::string> linkNames = {"base_link",
                                      "shoulder_link",
                                      "upper_arm_link",
                                      "forearm_link",
                                      "wrist_1_link",
                                      "wrist_2_link",
                                      "wrist_3_link"};

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
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)10);
    paramReferenceCorner = parameterHandler->AddParameter("ReferenceCorner", "", (int)4);
    paramCalibCount = parameterHandler->AddParameter("CalibCount", "", (int)5);
    paramCalibTagID = parameterHandler->AddParameter("CalibTargetID", "", 204);
    std::string defaultValue = "calib_target";
    paramCalibTargetTfName = parameterHandler->AddParameter("CalibTargetTfName", "", defaultValue);
    defaultValue = "depthcam1";
    paramCameraName = parameterHandler->AddParameter("CameraName", "", defaultValue);
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
    //get calib_target information
    geometry_msgs::TransformStamped calibTransform =  _transformHandler->GetTransform(paramCalibTargetTfName.GetValue(),
                                                                                      "base_link");

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
                for(int j = 0; j < newQRCodeData.size(); j++)
                {
                    newQRCodeData[j].calib_target = calibTransform;
                }

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

Mat lastRVec;
Mat lastTVec;
int calibRuns = 0;
bool calibrated = false;
void Calibrate()
{
    using namespace cv;
    using namespace std;

    // 2D/3D points
    vector<Point2d> imagePoints;
    vector<Point3d> modelPoints;
    int refernceCorner = paramReferenceCorner.GetValue();

    for(int i = 0; i < _calibTargets.size(); i++)
    {
        auto target = _calibTargets[i];

        imagePoints.push_back(Point2d(double(target.points[refernceCorner][0]),
                                       double(target.points[refernceCorner][1])));

        //ROS_INFO_STREAM(to_string(imagePoints[imagePoints.size()-1].x) + ", " +
        //                        to_string(imagePoints[imagePoints.size()-1].y));


        modelPoints.push_back(Point3d(target.calib_target.transform.translation.x,
                                      target.calib_target.transform.translation.y,
                                      target.calib_target.transform.translation.z));

        //ROS_INFO_STREAM(to_string(modelPoints[modelPoints.size()-1].x) + ", " +
        //                to_string(modelPoints[modelPoints.size()-1].y)+ ", " +
        //                to_string(modelPoints[modelPoints.size()-1].z));
        
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

    if (calibRuns == 0)
        solvePnP(modelPoints, imagePoints, intMat, Mat::zeros(4, 1, CV_64FC1), rVec, tVec, false, CV_EPNP);
    else
    {
        rVec = lastRVec;
        tVec = lastTVec;
        solvePnP(modelPoints, imagePoints, intMat, Mat::zeros(4, 1, CV_64FC1), rVec, tVec, true, CV_EPNP);
    }

    lastRVec = rVec;
    lastTVec = tVec;
    calibRuns += 1;

    ROS_INFO_STREAM(to_string(tVec.at<double>(0)) + ", " + to_string(tVec.at<double>(1)) + ", "
                    + to_string(tVec.at<double>(2)));

    //now we need the inverse matrix of the calculated transformation
    Mat rot, trans, jac;
    Rodrigues(rVec, rot, jac);

    //inverse of rotation = transpose!
    rot = rot.t();

    //inverse of translation
    trans = -rot * tVec;



    Eigen::Matrix3d mat;
    cv2eigen(rot, mat);

    Eigen::Quaternion<double> quat(mat);

    geometry_msgs::Pose calibPose;
    _lastCalibPose.position.x = trans.at<double>(0);
    _lastCalibPose.position.y = trans.at<double>(1);
    _lastCalibPose.position.z = trans.at<double>(2);

    _lastCalibPose.orientation.x = quat.x();
    _lastCalibPose.orientation.y = quat.y();
    _lastCalibPose.orientation.z = quat.z();
    _lastCalibPose.orientation.w = quat.w();

    _transformHandler->SendStaticTransform(calibPose, "base_link", paramCameraName.GetValue() + "_link");
    _transformHandler->WaitOne();

    calibrated = true;

    ROS_INFO_STREAM("Calibration result");
    ROS_INFO_STREAM(to_string(calibPose.position.x) + ", " + to_string(calibPose.position.y) + ", "
                    + to_string(calibPose.position.z));
    ROS_INFO_STREAM(to_string(calibPose.orientation.x) + ", " + to_string(calibPose.orientation.y) + ", "
                    + to_string(calibPose.orientation.z) + ", " + to_string(calibPose.orientation.w));


}

std::vector<Point3d> GetPose()
{
    std::vector<Point3d> ret;
    for (std::string link:linkNames)
    {
        geometry_msgs::Transform transform = _transformHandler->GetTransform(link, "base_link").transform;
        Point3d point(transform.translation.x, transform.translation.y, transform.translation.z);
        ret.push_back(point);
    }

    return ret;
}

void DrawPose(Mat image, std::vector<Point2d> points)
{
    for (cv::Point p:points)
    {
        cv::circle(image, p, 5, cv::Scalar(255, 0, 0), -1, cv::LINE_8, 0);
    }

}

void PublishReprojection()
{
    using namespace cv_bridge;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    CvImage imageBridge = CvImage(header, sensor_msgs::image_encodings::RGB8, GetCvImage());
    Mat intMat(3, 3, CV_64FC1, (void *) _currentCameraInfo.K.data());
    Mat distCoeffs(4, 1, CV_64FC1, (void *) _currentCameraInfo.D.data());
    std::vector<Point2d> points;
    std::vector<Point3d> originPoints = GetPose();

    projectPoints(originPoints, lastRVec, lastTVec, intMat, distCoeffs, points);
    DrawPose(imageBridge.image, points);

    sensor_msgs::Image imgMsg;
    imageBridge.toImageMsg(imgMsg);
    pubReporjectedImage.publish(imgMsg);

}


void SendCalibration()
{
    _transformHandler->SendTransform(_lastCalibPose, "base_link", paramCameraName.GetValue() + "_link");
    PublishReprojection();
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
                Point3d point(Point3d(newCodes[i].calib_target.transform.translation.x,
                                      newCodes[i].calib_target.transform.translation.y,
                                      newCodes[i].calib_target.transform.translation.z));

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

    qrCodesData = _poseDerivator->CalculateQRPose(qrCodesData, pclCloud, paramReferenceCorner.GetValue());
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

    _lastCalibPose.orientation.w = 1.0;

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

    _poseDerivator = new PoseDerivator(paramTagSize.GetValue());

    //define subcriber
    subCameraInfo = node->subscribe(node->resolveName("/depthcam/color/camera_info"), 1, cameraInfoCallback);
    ROS_INFO_STREAM("Listening to CameraInfo-Topic: " << subCameraInfo.getTopic());
    subImageMessage = node->subscribe(node->resolveName("/depthcam/color/image_raw"), 1, imageCallback);
    ROS_INFO_STREAM("Listening to RGBImage-Topic: " << subImageMessage.getTopic());
    subDepthImageMessage = node->subscribe(node->resolveName("/depthcam/depth_registered/points"), 1, depthCloudCallback);
    ROS_INFO_STREAM("Listening to DepthImage-Topic: " << subDepthImageMessage.getTopic());

    //define publisher
    pubScannedImage = node->advertise<sensor_msgs::Image>("ScannedImage", 100);
    ROS_INFO_STREAM("Will publish ScannedImages to " << pubScannedImage.getTopic());
    pubMarkedPointCloud = node->advertise<sensor_msgs::PointCloud2>("MarkedPointCloud", 100);
    ROS_INFO_STREAM("Will publish marked Pointclouds to " << pubMarkedPointCloud.getTopic());
    pubReporjectedImage =  node->advertise<sensor_msgs::Image>("ReprojectedPose", 100);
    ROS_INFO_STREAM("Will publish reprojected Points to " << pubReporjectedImage.getTopic());
    pubDebugPose = node->advertise<geometry_msgs::PoseArray>("DebugPose", 100);

    //launch transformation manager
    _transformManager = new TransformationManager(node, parameterHandler);
    _transformHandler = new helper::TransformationHandler(node);

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

            if(!calibrated)
            {
                for(int i = 0; i < processingFunctions.size(); i++)
                {
                    //execute added functions
                    (*processingFunctions[i])();
                }
            } else
            {
                SendCalibration();
            }

            rate.sleep();
        }
    }
    return 0;
}
