#ifndef QRCODEDATA_H
#define QRCODEDATA_H

#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>



class QRCodeData
{
public:

    enum TagType {QRCode, AprilTag, Unknown};

    QRCodeData();
    ~QRCodeData();

public:
    bool success = false;
    int id;
    int refrenceCounter = 1;
    TagType tagType = Unknown;
    std::string info = "";
    std::string frameName = "";
    std::string cameraFrameID = "world";
    std::vector<Eigen::Vector2i> points;
    std::vector<Eigen::Vector3f> points3D;
    geometry_msgs::Pose estimatedPose;
    float refSize = 0.0f;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;
    geometry_msgs::TransformStamped calib_target;

};

#endif // QRCODEDATA_H
