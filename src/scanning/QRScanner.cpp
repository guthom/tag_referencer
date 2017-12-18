#include "QRScanner.h"
#include <boost/algorithm/string.hpp>

QRScanner::QRScanner()
{
    Init();
}

void QRScanner::Init()
{
    //TODO: maybe theres a better parameterset for the configuration
    imgScanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
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

///Creates cv::Mat with the marked positiosns of the QRCode
/// \param qrCodesData vector of ImagaData object, one entry per qrCode
/// \param referenceCorner Number of the used reference corner for the final frame
/// \param image raw cv::Mat image to draw on
/// \return cv::Mat image with drawn qr code data
cv::Mat QRScanner::MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image)
{
    using namespace cv;

    if (qrCodesData.size() <= 0)
    {
        return image;
    }

    for (int i = 0; i < qrCodesData.size(); i++)
    {
        Point center = Point(qrCodesData[i].points[3][0], qrCodesData[i].points[3][1]);
        center.x += 25;
        center.y += 0;

        //set text with frame name
        std::string text = "ID:" + std::to_string(qrCodesData[i].id) + " " + qrCodesData[i].frameName;
        putText(image, text ,center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar( 255, 0, 0) ,2, LINE_AA);

        //visualize our Refernce
        center = Point(qrCodesData[i].points[referenceCorner][0], qrCodesData[i].points[referenceCorner][1]);
        circle(image, center, 15 , Scalar( 0, 0, 255), 4);

        for (int j = 0; j < qrCodesData[i].points.size(); j++)
        {
            center = Point(qrCodesData[i].points[j][0], qrCodesData[i].points[j][1]);
            circle(image, center, 5 , Scalar( 255, 0, 0), 4);

            putText(image, std::to_string(j)  ,center, FONT_HERSHEY_SIMPLEX, 0.8, Scalar( 255, 0, 0) ,2, LINE_AA);
            if(j < qrCodesData[i].points.size() -1)
            {
                Point p2 = Point(qrCodesData[i].points[j+1][0], qrCodesData[i].points[j+1][1]);
                line(image, center, p2, Scalar( 0, 255, 0), 2, 8);
            }
        }

        //draw last line
        Point p2 = Point(qrCodesData[i].points[0][0], qrCodesData[i].points[0][1]);
        line(image, center, p2, Scalar( 0, 255, 0), 2, 8);
    }

    return image;
}

void QRScanner::ProcessRawString(std::string rawString, QRCodeData* retData)
{
    std::vector<std::string> qrCode;
    boost::split(qrCode, rawString, boost::is_any_of(",;[]"));

    try
    {
        retData->id = std::stoi(qrCode[0]);
        retData->success = true;
        retData->info=qrCode[1];
        retData->frameName=qrCode[2];
        //parse framePose given within the qrcode information
        retData->framePose.position.x = std::stof(qrCode[3]);
        retData->framePose.position.y = std::stof(qrCode[4]);
        retData->framePose.position.z = std::stof(qrCode[5]);
        retData->framePose.orientation.x = std::stof(qrCode[6]);
        retData->framePose.orientation.y = std::stof(qrCode[7]);
        retData->framePose.orientation.z = std::stof(qrCode[8]);
        retData->framePose.orientation.w = std::stof(qrCode[9]);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while parsing QR-Code Information, wrong format???");
    }
}

/// Scannes an cv::Mat image for QRCodes
/// \param cvImage input cv::Mat Image to scan
/// \return vector of ImageData objects, one object per QRCode
std::vector<QRCodeData> QRScanner::ScanCurrentImg(cv::Mat cvImage)
{
    std::vector<QRCodeData> qrCodes;

    auto image = CreateZBarImage(&cvImage);

    int res = imgScanner.scan(*image);

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
