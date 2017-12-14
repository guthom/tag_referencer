#ifndef POSEDERIVATOR_H
#define POSEDERIVATOR_H

#include <vector>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include "../data/QRCodeData.h"

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud;

class PoseDerivator
{
public:
    PoseDerivator();
    ~PoseDerivator();

    std::vector<QRCodeData> CalculateQRPose(std::vector<QRCodeData> qrCodes, PointCloud pointCloud, int referencePoint);

private:

    void Init();

    geometry_msgs::Transform PoseToTransform(geometry_msgs::Pose pose);
};

#endif // POSEDERIVATOR_H
