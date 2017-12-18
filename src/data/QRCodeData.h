#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

class QRCodeData
{
public:
    QRCodeData();

    ~QRCodeData();

public:
    bool success = false;
    int id;
    int refrenceCounter = 1;
    std::string info = "";
    std::string frameName = "";
    std::string cameraFrameID = "world";
    std::vector<Eigen::Vector2i> points;
    std::vector<Eigen::Vector3f> points3D;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;

};

#endif // IMAGEDATA_H
