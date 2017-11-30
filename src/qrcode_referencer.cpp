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

//common stuff
std::string nodeName = "qrcode_referencer";

//ros sutff
ros::NodeHandle* node;
ros::Subscriber subImageMessage;

//parameter stuff
customparameter::ParameterHandler* parameterHandler;
customparameter::Parameter<int> paramRefreshRate;

//zbar stuff
using namespace zbar;
ImageScanner imgScanner;

struct image_data{
    bool success;
    int id;
    std::string info;
    std::string frameName;
    geometry_msgs::Pose framePose;
    geometry_msgs::Pose qrPose;

};

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
    imgScanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);


}

void Init()
{
    InitParams();
    InitZBar();
}

image_data processRawString(std::string rawData)
{
    image_data ret;
    std::vector<std::string> vector;
    boost::split(vector, rawData, boost::is_any_of(";,[]"));

    try {
        ret.id = std::stoi(vector[0]);
        ret.info=vector[1];
        ret.frameName=vector[2];
        //parse framePose given within the qrcode information
        ret.framePose.position.x = std::stof(vector[4]);
        ret.framePose.position.y = std::stof(vector[5]);
        ret.framePose.position.z = std::stof(vector[6]);
        ret.framePose.orientation.x = std::stof(vector[7]);
        ret.framePose.orientation.y = std::stof(vector[8]);
        ret.framePose.orientation.z = std::stof(vector[9]);
        ret.framePose.orientation.w = std::stof(vector[10]);
    } catch (const std::exception& e)
    {
        ROS_ERROR_STREAM("Error while parsing QR-Code Information, wrong format???");
    }

    return ret;
}

image_data ScanImg(Image* image)
{
    image_data qrCode;
    qrCode.success = false;

    int res = imgScanner.scan(*image);
    ROS_INFO_STREAM("Result:" << res);

    if (res != 0)
    {
        std::string rawText;
        for(Image::SymbolIterator symbol = image->symbol_begin(); symbol != image->symbol_end(); ++symbol)
        {
            rawText += symbol->get_data().c_str();
        }

        qrCode = processRawString(rawText);
        qrCode.success = true;
    }
    // clean up
    image->set_data(NULL, 0);

    ROS_INFO_STREAM(qrCode.frameName);
    return qrCode;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg, "mono8");

    Image* zbar_image = new Image((unsigned  int)cv_image->image.cols, (unsigned  int)cv_image->image.rows, "Y800",
                     cv_image->image.data, (unsigned  int)(cv_image->image.cols * cv_image->image.rows));

    image_data qrCode = ScanImg(zbar_image);
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
