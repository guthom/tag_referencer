#include "PoseDerivator.h"
#include <Eigen/Dense>

PoseDerivator::PoseDerivator(float tagSize) : tagSize(tagSize)
{
    Init();
}

void PoseDerivator::Init()
{
    //init static object points
    staticObjectPoints.push_back(cv::Point3d(0.0, 0.0, 0.0));
    staticObjectPoints.push_back(cv::Point3d(0.0, tagSize, 0.0));
    staticObjectPoints.push_back(cv::Point3d(tagSize, tagSize, 0.0));
    staticObjectPoints.push_back(cv::Point3d(tagSize, 0.0, 0.0));
}

/// Calculates the Orientation of the QR-Code by calcualting a plane with the given LGS
/// The orientation will allways be facing from corner 1 to corner 0
/// 3 - 2
/// | 5 |
/// 0 - 1
/// \param points, given points to derive the plane in R3
/// \return
geometry_msgs::Quaternion CalculateOrientation(std::vector<Eigen::Vector3f> points)
{
    using namespace Eigen;
    geometry_msgs::Quaternion ret;

    Vector3f vec1 = points[3] - points[0];
    vec1.normalize();
    Vector3f vec2 = points[1] - points[0];
    vec2.normalize();
    Vector3f vec3 = vec1.cross(vec2);
    vec3.normalize();


    Quaternionf quat, quat2, quat3;
    quat.setFromTwoVectors(vec1, vec2);
    quat.normalize();
    quat2.setFromTwoVectors(vec1, vec3);
    quat2.normalize();

    //quat = quat2;



    //calculate unit vectors of new coordinate system
    Vector3f xVec = points[3] - points[0];
    xVec.normalize();
    Vector3f yVec = points[1] - points[0];
    yVec.normalize();
    //Calculate Z Unit Vector by normalize cross product of other x/y vectors -> normal for x/y plane
    Vector3f zVec = xVec.cross(yVec);
    zVec.normalize();

    Eigen::Matrix3f mat;

    mat <<  xVec[0],  yVec[0],  zVec[0],
            xVec[1],  yVec[1],  zVec[1],
            xVec[2],  yVec[2],  zVec[2];
    Eigen::Quaternionf test(mat);
    test.normalize();


    Eigen::Quaternionf baseQuat(0.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Quaternionf rotation(0.707f, 0.0f, 0.0f, -0.707f);

    baseQuat = test;

    ret.x = baseQuat.x();
    ret.y = baseQuat.y();
    ret.z = baseQuat.z();
    ret.w = baseQuat.w();

    return ret;
}

std::vector<QRCodeData> PoseDerivator::CalculateQRPose(std::vector<QRCodeData> qrCodesData,
                                                       PointCloud pointCloud, int referenceCorner)
{
    if(pointCloud.data.size() > 0)
    {
        for (int i = 0; i < qrCodesData.size(); i++)
        {
            for (int j = 0; j < qrCodesData[i].points.size(); j++)
            {
                int arrayPosition = qrCodesData[i].points[j][1] * pointCloud.row_step +
                                    qrCodesData[i].points[j][0]* pointCloud.point_step;

                int arrayPosX = arrayPosition + pointCloud.fields[0].offset; // X has an offset of 0
                int arrayPosY = arrayPosition + pointCloud.fields[1].offset; // Y has an offset of 4
                int arrayPosZ = arrayPosition + pointCloud.fields[2].offset; // Z has an offset of 8

                Eigen::Vector3f point;

                memcpy(&point[0], &pointCloud.data[arrayPosX], sizeof(float));
                memcpy(&point[1], &pointCloud.data[arrayPosY], sizeof(float));
                memcpy(&point[2], &pointCloud.data[arrayPosZ], sizeof(float));

                qrCodesData[i].points3D.push_back(point);
                //add center as fith point
            }

            //set position of pose to choosen reference point
            qrCodesData[i].qrPose.position.x = qrCodesData[i].points3D[referenceCorner][0];
            qrCodesData[i].qrPose.position.y = qrCodesData[i].points3D[referenceCorner][1];
            qrCodesData[i].qrPose.position.z = qrCodesData[i].points3D[referenceCorner][2];
            qrCodesData[i].qrPose.orientation = CalculateOrientation(qrCodesData[i].points3D);
        }

    }
    return qrCodesData;
}

std::vector<QRCodeData> PoseDerivator::CalculateQRPose(std::vector<QRCodeData> qrCodesData,
                                                       PointCloud pointCloud, int referenceCorner,
                                                        sensor_msgs::CameraInfo cameraInfo)
{

    qrCodesData = this->CalculateQRPose(qrCodesData, pointCloud, referenceCorner);

    for (int i = 0; i < qrCodesData.size(); i++)
    {
        std::vector<cv::Point2d> imagePoints;

        for (int j = 0; j < qrCodesData[i].points.size() - 1; j++)
        {
            imagePoints.emplace_back(cv::Point2d(qrCodesData[i].points[j][0], qrCodesData[i].points[j][1]));
        }

        auto pose = this->Estimate3DTagPose(staticObjectPoints, imagePoints, cameraInfo);
        qrCodesData[i].estimatedPose = pose;
    }

    return qrCodesData;
}

geometry_msgs::Pose
PoseDerivator::Estimate3DTagPose(std::vector<cv::Point3d> objectPoints, std::vector<cv::Point2d> imagePoints,
                                 sensor_msgs::CameraInfo cameraInfo) {
    cv::Mat rvec, tvec;

    cv::Matx33d cameraMatrix(cameraInfo.K.data());

    cv::Vec4f distCoeffs(0,0,0,0); // assume no distortion ;-)
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Matx33d R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d rotMat;
    rotMat <<
            R(0,0), R(0,1), R(0,2),
            R(1,0), R(1,1), R(1,2),
            R(2,0), R(2,1), R(2,2);

    Eigen::Quaterniond rot(rotMat);
    rot.normalize();
    geometry_msgs::Pose ret;

    ret.orientation.w = rot.w();
    ret.orientation.x = rot.x();
    ret.orientation.y = rot.y();
    ret.orientation.z = rot.z();
    ret.position.x =  tvec.at<double>(0);
    ret.position.y =  tvec.at<double>(1);
    ret.position.z =  tvec.at<double>(2);

    return ret;
}


PoseDerivator::~PoseDerivator()
{

}

