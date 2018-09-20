#ifndef APRILTAGSCANNER_H
#define APRILTAGSCANNER_H

#include <string.h>

#include "ScannerBase.h"

#include <custom_parameter/parameter.h>

using namespace cv;
using namespace customparameter;

class AprilTagScanner
        : public ScannerBase {
    public:
        AprilTagScanner(customparameter::ParameterHandler* paramHandler);
        ~AprilTagScanner();

        virtual std::vector<QRCodeData> ScanCurrentImg(Mat cvImage);
        virtual cv::Mat MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image);

    private:

        void Init();
        void InitParams();
        Parameter<std::string> paramTagFamily;
        Parameter<int> paramTagBoarder;
        Parameter<int> paramTagThreads;
        Parameter<bool> paramRefineEdges;
        Parameter<bool> paramRefineDecode;
        Parameter<bool> paramRefinePose;
        Parameter<float> paramTagDecimate;
        Parameter<float> paramTagBlur;

};

#endif // APRILTAGSCANNER_H
