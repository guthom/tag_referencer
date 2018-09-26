#include "QRScanner.h"
#include <boost/algorithm/string.hpp>

QRScanner::QRScanner(customparameter::ParameterHandler* paramHandler) : ScannerBase(paramHandler)
{
    Init();
}

void QRScanner::Init()
{
    InitParams();
    //TODO: maybe theres a better parameterset for the configuration
    imgScanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
}

void QRScanner::InitParams()
{
    //init params
    std::string subNamespace = "QRCodeScanner/";
    //Standard params
    paramReferenceSize = _paramHandler->AddParameter(subNamespace + "ReferenceSize", "", (float)0.0f);

}

zbar::Image* QRScanner::CreateZBarImage(cv::Mat* image)
{
    //cv::namedWindow("test1", cv::WINDOW_AUTOSIZE);
    //cv::imshow("test1", image);

    //convert to GrayScale first
    cv::Mat grayImage;
    cv::cvtColor(*image, grayImage, CV_RGB2GRAY);

    //v::namedWindow("test2", cv::WINDOW_AUTOSIZE);
    //cv::imshow("test2", grayImage);
    //waitKey(0);

    //TODO: Check if memory leaks!
    zbar::Image* zbarImage = new Image(grayImage.cols, grayImage.rows, "Y800", grayImage.data, (grayImage.cols * grayImage.rows));

    return zbarImage;
}

void QRScanner::ProcessRawString(std::string rawString, QRCodeData* retData)
{
    std::vector<std::string> qrCode;
    boost::split(qrCode, rawString, boost::is_any_of(",;[]"));

    if (qrCode.size() > 1)
    {
        try
        {
            retData->id = std::stoi(qrCode[0]);
            retData->success = true;
            retData->frameName=qrCode[1];
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Error while parsing QR-Code Information, wrong format???");
        }
    }
}

/// Scannes an cv::Mat image for QRCodes
/// \param cvImage input cv::Mat Image to scan
/// \return vector of ImageData objects, one object per QRCode
std::vector<QRCodeData> QRScanner::ScanCurrentImg(cv::Mat cvImage)
{
    std::vector<QRCodeData> qrCodes;

    //need to copy cv-matrix before we're able to scann it, it can happen, that the reference is getting lost
    //and we run in a segfault
    cv::Mat copyImg = cvImage.clone();

    auto image = CreateZBarImage(&copyImg);

    int res = 0;
    try
    {
        res = imgScanner.scan(*image);
    }
    catch (std::exception& e)
    {
    }

    if(res < 0)
    {
        ROS_INFO_STREAM("Error occurred while scanning image! Scanning Result = " + std::to_string(res));
    }
    else if (res >= 0)
    {
        //extract symbol information
        int counter = 0;
        for(Image::SymbolIterator symbol = image->symbol_begin(); symbol !=  image->symbol_end(); ++symbol)
        {
            QRCodeData entry;

            entry.tagType = tagType;
            entry.refSize = paramReferenceSize.GetValue();

            ProcessRawString(symbol->get_data().c_str(), &entry);

            //run through all detected points
            std::vector<Eigen::Vector2i> points;
            for(int i = 0; i < symbol->get_location_size(); i++)
            {
                Eigen::Vector2i point;
                point[0] = symbol->get_location_x(i);
                point[1] = symbol->get_location_y(i);
                points.push_back(point);
            }

            points.push_back(CalculateCenter(points));
            entry.points = points;

            qrCodes.push_back(entry);
            counter += 1;
        }
    }
    //imgScanner.recycle_image(*image);
    return qrCodes;
}

QRScanner::~QRScanner()
{

}
