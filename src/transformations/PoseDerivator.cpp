#include "PoseDerivator.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <math.h>
#include <tf/LinearMath/Quaternion.h>

PoseDerivator::PoseDerivator()
{
}

void PoseDerivator::Init()
{

}

/// Calculates the Orientation of the QR-Code by calcualting a plane with the given LGS
/// The orientation will allways be facing from corner 1 to corner 0
/// 0 -  3
/// | QR |
/// 1 -  2
/// \param points, given points to derive the plane in R3
/// \return
geometry_msgs::Quaternion CalculateOrientation(std::vector<Eigen::Vector3f> points)
{
    using namespace Eigen;
    geometry_msgs::Quaternion ret;

    //calculate unit vectors of new coordinate system
    Vector3f xVec = points[2] - points[1];
    xVec.normalize();
    Vector3f yVec = points[0] - points[1];
    yVec.normalize();
    //Calculate Z Unit Vector by normalize cross product of other x/y vectors -> normal for x/y plane
    Vector3f zVec = -xVec.cross(yVec);
    zVec.normalize();

    float rot1, rot2, rot3;
    rot1 = acosf(Vector3f::UnitX().dot(xVec));
    rot2 = acosf(Vector3f::UnitY().dot(yVec));
    rot3 = acosf(Vector3f::UnitZ().dot(zVec));

    Eigen::Quaternionf  quat =  AngleAxisf(rot1, Vector3f::UnitZ()) *
                                AngleAxisf(rot2, Vector3f::UnitY()) *
                                AngleAxisf(rot3, Vector3f::UnitZ());

    ret.x = quat.x();
    ret.y = quat.y();
    ret.z = quat.z();
    ret.w = quat.w();

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

PoseDerivator::~PoseDerivator()
{

}
