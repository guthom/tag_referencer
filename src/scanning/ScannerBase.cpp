//
// Created by thomas on 13.04.18.
//

#include "ScannerBase.h"

ScannerBase::ScannerBase(customparameter::ParameterHandler* paramHandler) : _paramHandler(paramHandler)
{

}


///Creates cv::Mat with the marked positiosns of the QRCode
/// \param qrCodesData vector of ImagaData object, one entry per qrCode
/// \param referenceCorner Number of the used reference corner for the final frame
/// \param image raw cv::Mat image to draw on
/// \return cv::Mat image with drawn qr code data
cv::Mat ScannerBase::MarkImage(std::vector<QRCodeData> qrCodesData, int referenceCorner, cv::Mat image)
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