#ifndef QRSCANNER_H
#define QRSCANNER_H

#include <string.h>
#include "zbar.h"
#include "ScannerBase.h"

using namespace zbar;
using namespace cv;

class QRScanner
        : public ScannerBase {
    public:
        QRScanner(customparameter::ParameterHandler* paramHandler);
        ~QRScanner();

        virtual std::vector<QRCodeData> ScanCurrentImg(Mat cvImage);
        virtual cv::Mat MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image);

    private:
        //zbar stuf
        ImageScanner imgScanner;

        void Init();

        void InitParams();

        void ProcessRawString(std::string rawString, QRCodeData* retData);
        zbar::Image* CreateZBarImage(cv::Mat* image);
};

#endif // SCANNER_H
