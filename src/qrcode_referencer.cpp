#include <ros/ros.h>
#include <string.h>

#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <custom_parameter/parameterHandler.h>
#include <custom_parameter/parameter.h>

#include "zbar.h"

#include <boost/thread.hpp>

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

std::string ScanImg(Image* image)
{
    std::string qrCode;
    int res = imgScanner.scan(*image);
    ROS_INFO_STREAM("Result:" << res);

    if (res != 0)
    {
        for(Image::SymbolIterator symbol = image->symbol_begin(); symbol != image->symbol_end(); ++symbol)
        {
            qrCode += symbol->get_data().c_str();
        }

        bool test = true;

    }
    // clean up
    image->set_data(NULL, 0);

    return qrCode;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_image;
    cv_image = cv_bridge::toCvShare(msg, "mono8");

    Image* zbar_image = new Image((unsigned  int)cv_image->image.cols, (unsigned  int)cv_image->image.rows, "Y800",
                     cv_image->image.data, (unsigned  int)(cv_image->image.cols * cv_image->image.rows));

    std::string qrCode = ScanImg(zbar_image);
    ROS_INFO_STREAM(qrCode);
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
