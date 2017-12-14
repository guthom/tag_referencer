#ifndef QRSCANNER_H
#define QRSCANNER_H

#include <ros/ros.h>
#include <string.h>
#include "zbar.h"

#include "../data/ImageData.h"

using namespace zbar;
using namespace cv;

class QRScanner
{
    public:
        QRScanner();
        ~QRScanner();

        std::vector<ImageData> ScanCurrentImg(Mat cvImage);
        cv::Mat MarkImage(std::vector<ImageData> qrCodesData, int referenceCorner, cv::Mat image);

    private:
        //zbar stuf
        ImageScanner imgScanner;

        void Init();

        void ProcessRawString(std::string rawString, ImageData* retData);
        zbar::Image* CreateZBarImage(cv::Mat* image);
};

#endif // SCANNER_H
