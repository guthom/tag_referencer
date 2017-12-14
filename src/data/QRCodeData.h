#ifndef IMAGEDATA_H
#define IMAGEDATA_H

#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Pose.h>

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
    std::vector<cv::Point2i> points;
    std::vector<pcl::PointXYZ> points3D;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;

};

#endif // IMAGEDATA_H
