#ifndef APRILTAGSCANNER_H
#define APRILTAGSCANNER_H

#include <string.h>

#include "ScannerBase.h"

#include <custom_parameter/parameter.h>

#include <apriltag.h>

using namespace cv;
using namespace customparameter;


class AprilTagScanner
        : public ScannerBase {
    public:
        AprilTagScanner(customparameter::ParameterHandler* paramHandler);
        ~AprilTagScanner();

        virtual std::vector<QRCodeData> ScanCurrentImg(Mat cvImage);

    private:

        QRCodeData::TagType tagType = QRCodeData::TagType::AprilTag;

        void Init();

        //ParamStuff
        void InitParams();
        Parameter<std::string> paramTagFamily;
        Parameter<int> paramTagBorder;
        Parameter<int> paramTagThreads;
        Parameter<bool> paramRefineEdges;
        Parameter<bool> paramRefineDecode;
        Parameter<bool> paramRefinePose;
        Parameter<float> paramTagDecimate;
        Parameter<float> paramTagSigma;
        Parameter<std::string> paramTagPrefix;

        //AprilTags stuff
        apriltag_family *atFamily;
        apriltag_detector *atDetector;

        void CheckTagFamily();
        void ReleaseTagFamily();
        void InitDetector();

};

#endif // APRILTAGSCANNER_H
