#ifndef POSEDERIVATOR_H
#define POSEDERIVATOR_H

#include <vector>

#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>

#include "../data/QRCodeData.h"

class PoseDerivator
{
public:
    PoseDerivator();
    ~PoseDerivator();

    geometry_msgs::Pose CalculateQRPose(std::vector<QRCodeData> qrCodes);
    geometry_msgs::Transform CalculateQRTransform();

private:

    void Init();

    geometry_msgs::Transform PoseToTransform(geometry_msgs::Pose pose);


};

#endif // POSEDERIVATOR_H
