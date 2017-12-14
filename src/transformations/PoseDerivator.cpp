#include "PoseDerivator.h"

PoseDerivator::PoseDerivator()
{
}

void PoseDerivator::Init()
{

}

std::vector<QRCodeData> PoseDerivator::CalculateQRPose(std::vector<QRCodeData> qrCodesData,
                                                   PointCloud pointCloud, int referencePoint)
{
    if(pointCloud.data.size() > 0)
    {
        for (int i = 0; i < qrCodesData.size(); i++)
        {
            for (int j = 0; j < qrCodesData[i].points.size(); j++)
            {
                int arrayPosition = qrCodesData[i].points[j].y * pointCloud.row_step +
                                    qrCodesData[i].points[j].x * pointCloud.point_step;

                int arrayPosX = arrayPosition + pointCloud.fields[0].offset; // X has an offset of 0
                int arrayPosY = arrayPosition + pointCloud.fields[1].offset; // Y has an offset of 4
                int arrayPosZ = arrayPosition + pointCloud.fields[2].offset; // Z has an offset of 8

                pcl::PointXYZ point;

                memcpy(&point.x, &pointCloud.data[arrayPosX], sizeof(float));
                memcpy(&point.y, &pointCloud.data[arrayPosY], sizeof(float));
                memcpy(&point.z, &pointCloud.data[arrayPosZ], sizeof(float));

                qrCodesData[i].points3D.push_back(point);
            }

            //set position of pose to choosen reference point
            qrCodesData[i].qrPose.position.x = qrCodesData[i].points3D[referencePoint].x;
            qrCodesData[i].qrPose.position.y = qrCodesData[i].points3D[referencePoint].y;
            qrCodesData[i].qrPose.position.z = qrCodesData[i].points3D[referencePoint].z;
            qrCodesData[i].qrPose.orientation.w = 1.0;
        }
    }
    return qrCodesData;
}

PoseDerivator::~PoseDerivator()
{

}
