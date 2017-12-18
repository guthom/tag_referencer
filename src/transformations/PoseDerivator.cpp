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

    /*
    //use this if plane/normal will be used for orientation otherwise we can easy create the quaternion with 2 points:-o
    //fitting plane -> n*p+d=0 -> normal * position + offset
    //to solve: ax + by + cz + d = 0
    MatrixXf leftSide(points.size(), 3);
    MatrixXf rightSide(points.size(), 1);

    //feed left and right hand side with points
    for (int i = 0; i < points.size(); i++)
    {
        leftSide(i,0) = points[i][0];
        leftSide(i,1) = points[i][1];
        leftSide(i,2) = points[i][2];
        rightSide(i,0) = -1.0f;
    }

    const IOFormat fmt(2, DontAlignCols, "\t", " ", "", "", "", "");

    //calculate the planes normal vecotr by solving the constructed LGS
    Vector3f normalVec = leftSide.jacobiSvd(ComputeThinU | ComputeThinV).solve(rightSide);
    std::cout << normalVec.format(fmt)  << std::endl;
    */

    Vector3f xVec = points[2] - points[1];
    xVec.normalize();
    Vector3f yVec = points[0] - points[1];
    yVec.normalize();

    float rot1, rot2, rot3;
    rot1 = atan2(xVec[1], xVec[0]);
    rot2 = atan2(yVec[2], yVec[0]);
    rot3 = 0.0;

    //rotate with ZXZ convention
    Eigen::Quaternionf  quat =  AngleAxisf(rot1, Vector3f::UnitZ()) *
                                AngleAxisf(rot2, Vector3f::UnitX()) *
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
