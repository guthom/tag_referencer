#include "AprilTagScanner.h"
#include <boost/algorithm/string.hpp>

#include "common/homography.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include "tag25h9.h"
#include "tag25h7.h"
#include "tag16h5.h"

AprilTagScanner::AprilTagScanner(customparameter::ParameterHandler* paramHandler) : ScannerBase(paramHandler)
{
    Init();
}

void AprilTagScanner::Init()
{
    InitParams();

    CheckTagFamily();
    InitDetector();
}


void AprilTagScanner::InitDetector()
{
    atDetector = apriltag_detector_create();
    atFamily->black_border = (uint32_t)paramTagBorder.GetValue();

    apriltag_detector_add_family(atDetector, atFamily);
    atDetector->quad_decimate = paramTagDecimate.GetValue();
    atDetector->quad_sigma = paramTagSigma.GetValue();
    atDetector->nthreads = paramTagThreads.GetValue();
    atDetector->debug = false;
    atDetector->refine_edges = paramRefineEdges.GetValue();
    atDetector->refine_decode = paramRefineDecode.GetValue();
    atDetector->refine_pose = paramRefinePose.GetValue();
}

void AprilTagScanner::CheckTagFamily()
{
    auto family = paramTagFamily.GetValue();

    if (family == "tag36h11")
    {
        atFamily = tag36h11_create();
        return;
    }

    if (family == "tag36h10")
    {
        atFamily = tag36h10_create();
        return;
    }

    if (family == "tag25h9")
    {
        atFamily = tag25h9_create();
        return;
    }

    if (family == "tag25h7")
    {
        atFamily = tag25h7_create();
        return;
    }
    if (family == "tag16h5")
    {
        atFamily = tag16h5_create();
        return;
    }

    ROS_ERROR_STREAM("Don't know the given TagFamily: " + family + " Will exit now!");
    exit(1);
}

void AprilTagScanner::InitParams()
{
    //init params
    std::string subNamespace = "ApriTagScanner/";
    //Standard params
    paramReferenceSize = _paramHandler->AddParameter(subNamespace + "ReferenceSize", "", (float)0.0f);
    paramTagFamily = _paramHandler->AddParameter(subNamespace + "TagFamily", "", (std::string)"tag36h11");
    paramTagBorder = _paramHandler->AddParameter(subNamespace + "TagBorder", "", 1);
    paramTagThreads = _paramHandler->AddParameter(subNamespace + "TagThreads", "", 2);
    paramRefineEdges = _paramHandler->AddParameter(subNamespace + "RefineEdges", "", true);
    paramRefineDecode = _paramHandler->AddParameter(subNamespace + "RefineDecode", "", false);
    paramRefinePose = _paramHandler->AddParameter(subNamespace + "RefinePose", "", false);
    paramTagDecimate = _paramHandler->AddParameter(subNamespace + "TagDecimate", "", (float)1.0f);
    paramTagSigma = _paramHandler->AddParameter(subNamespace + "TagBlur", "", (float)0.0f);
}

/// Scannes an cv::Mat image for QRCodes
/// \param cvImage input cv::Mat Image to scan
/// \return vector of ImageData objects, one object per QRCode
std::vector<QRCodeData> AprilTagScanner::ScanCurrentImg(cv::Mat cvImage)
{
    std::vector<QRCodeData> aprilTags;

    int res = 0;
    try
    {
        //need to copy cv-matrix before we're able to scann it, it can happen, that the reference is getting lost
        //and we run in a segfault
        cv::Mat copyImg = cvImage.clone();
        cv::Mat grayImg;
        cv::cvtColor(copyImg, grayImg, CV_BGR2GRAY);

        image_u8_t atImage = { .width = grayImg.cols,
                .height = grayImg.rows,
                .stride = grayImg.cols,
                .buf = grayImg.data
        };


        zarray* rawDetections = apriltag_detector_detect(atDetector, &atImage);

        for (int i=0; i < zarray_size(rawDetections); i++)
        {
            QRCodeData data;
            data.tagType = tagType;
            data.refSize = paramReferenceSize.GetValue();

            apriltag_detection_t *detection;
            zarray_get(rawDetections, i, &detection);

            data.id = detection->id;
            data.frameName = "boxApril_" + std::to_string(data.id);

            std::vector<Eigen::Vector2i> points;

            for(int i = 0; i < 4; i++)
            {
                Eigen::Vector2i point;
                point[0] = (int)detection->p[i][0];
                point[1] = (int)detection->p[i][1];
                points.push_back(point);
            }

            points.push_back(CalculateCenter(points));
            data.points = points;

            aprilTags.push_back(data);
        }


        zarray_destroy(rawDetections);
    }
    catch (std::exception& e) {
    }

    return aprilTags;
}

void AprilTagScanner::ReleaseTagFamily()
{
    auto family = atFamily->name;

    if (family == "tag36h11")
    {
        tag36h11_destroy(atFamily);
        return;
    }

    if (family == "tag36h10")
    {
        tag36h10_destroy(atFamily);
        return;
    }

    if (family == "tag25h9")
    {
        tag25h9_destroy(atFamily);
        return;
    }

    if (family == "tag25h7")
    {
        tag25h7_destroy(atFamily);
        return;
    }
    if (family == "tag16h5")
    {
        tag16h5_destroy(atFamily);
        return;
    }
}

AprilTagScanner::~AprilTagScanner()
{
    //free used resources
    apriltag_detector_destroy(atDetector);
    ReleaseTagFamily();

}
