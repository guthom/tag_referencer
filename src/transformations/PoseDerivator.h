#ifndef POSEDERIVATOR_H
#define POSEDERIVATOR_H

#include <vector>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#include "../data/QRCodeData.h"
#include <string>
#include <queue>
#include <sensor_msgs/CameraInfo.h>
#include <map>
#include <Eigen/Dense>
#include <cv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud;

class PoseDerivator
{
public:
    PoseDerivator(float tagSize);
    ~PoseDerivator();

    geometry_msgs::Pose Estimate3DTagPose(std::vector<cv::Point3d> objectPoints,
                                      std::vector<cv::Point2d> imagePoints,
                                      sensor_msgs::CameraInfo cameraInfo);

    std::vector<QRCodeData> CalculateQRPose(std::vector<QRCodeData> qrCodes, PointCloud pointCloud, int referenceCorner);
    std::vector<QRCodeData> CalculateQRPose(std::vector<QRCodeData> qrCodes, PointCloud pointCloud, int referenceCorner,
                                            sensor_msgs::CameraInfo cameraInfo);

private:
    std::vector<cv::Point3d> staticObjectPoints;
    float tagSize;
    void Init();


};

#endif // POSEDERIVATOR_H
