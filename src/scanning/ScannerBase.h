//
// Created by thomas on 13.04.18.
//

#ifndef PROJECT_SCANNERBASE_H
#define PROJECT_SCANNERBASE_H

#include <ros/ros.h>
#include <opencv/cv.hpp>
#include "src/data/QRCodeData.h"
#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>


class ScannerBase
{
public:
    ScannerBase(customparameter::ParameterHandler* paramHandler);


    virtual std::vector<QRCodeData> ScanCurrentImg(cv::Mat cvImage) = 0;
    cv::Mat static MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image);

protected:

    QRCodeData::TagType tagType = QRCodeData::TagType::Unknown;

    customparameter::ParameterHandler* _paramHandler;
    customparameter::Parameter<float> paramReferenceSize;

    Eigen::Vector2i CalculateCenter(std::vector<Eigen::Vector2i> points);

};



#endif //PROJECT_SCANNERBASE_H
