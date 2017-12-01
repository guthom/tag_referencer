#include <ros/ros.h>
#include <string.h>

#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "zbar.h"

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

struct image_data{
    bool success = false;
    int id;
    std::string info = "";
    std::string frameName = "";
    int pixX = 0;
    int pixY = 0;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;
};

//common stuff
std::string nodeName = "qrcode_referencer";

//boost stuff
boost::mutex* _mutex;

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subImageMessage;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<int> paramRefreshRate;

//zbar stuff
using namespace zbar;
ImageScanner imgScanner;

//qrCode data
std::vector<image_data>* _qrCodesData;
void setQrCodesData(std::vector<image_data> data)
{
    _mutex->lock();
    _qrCodesData = &data;
    _mutex->unlock();
}


void InitParams()
{
    //init params
    parameterHandler = new customparameter::ParameterHandler(node);
    std::string subNamespace = "";
    //Standard params
    paramRefreshRate = parameterHandler->AddParameter("RefreshRate", "", (int)20);
}

void InitZBar()
{
    //TODO: maybe theres a better parameterset for the configuration
    imgScanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
}

void Init()
{
    _mutex = new boost::mutex();
    InitParams();
    InitZBar();
}

void ProcessRawString(std::string rawString, image_data* retData)
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

void ScanImg(Image* image)
{
    std::vector<image_data> qrCodes;

    int res = imgScanner.scan(*image);
    ROS_INFO_STREAM("Result:" << res);

    if (res != 0)
    {
        //extract symbol information
        int counter = 0;
        for(Image::SymbolIterator symbol = image->symbol_begin(); symbol != image->symbol_end(); ++symbol)
        {
            std::string test = symbol->get_data();

            image_data entry;
            ProcessRawString(symbol->get_data().c_str(), &entry);

            entry.pixX = symbol->get_location_x(counter);
            entry.pixY = symbol->get_location_y(counter);

            qrCodes.push_back(entry);
            counter += 1;
        }
    }
    // clean up
    image->set_data(NULL, 0);
    for(int i = 0; i < qrCodes.size(); i++) {
        ROS_INFO_STREAM(qrCodes[i].frameName + "; "<< qrCodes[i].pixY << "; " << qrCodes[i].pixX);
    }

    setQrCodesData(qrCodes);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg, "mono8");

    Image* zbar_image = new Image((unsigned  int)cv_image->image.cols, (unsigned  int)cv_image->image.rows, "Y800",
                     cv_image->image.data, (unsigned  int)(cv_image->image.cols * cv_image->image.rows));

    ScanImg(zbar_image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, nodeName);
    node = new ros::NodeHandle(nodeName);

    Init();

    subImageMessage = node->subscribe("/zed/right/image_rect_color", 1000, imageCallback);

    ros::Rate rate(paramRefreshRate.GetValue());
    while(node->ok())
    {        
        ros::spinOnce();

        rate.sleep();        
    }
    return 0;
}
