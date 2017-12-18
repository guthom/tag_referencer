#ifndef QRSCANNER_H
#define QRSCANNER_H

#include <ros/ros.h>
#include <string.h>
#include "zbar.h"

#include <opencv/cv.hpp>
#include "src/data/QRCodeData.h"

using namespace zbar;
using namespace cv;

class QRScanner
{
    public:
        QRScanner();
        ~QRScanner();

        std::vector<QRCodeData> ScanCurrentImg(Mat cvImage);
        cv::Mat MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image);

    private:
        //zbar stuf
        ImageScanner imgScanner;

        void Init();

        void ProcessRawString(std::string rawString, QRCodeData* retData);
        zbar::Image* CreateZBarImage(cv::Mat* image);
};

#endif // SCANNER_H
