//
// Created by thomas on 14.06.18.
//
#pragma once

#include <cstring>
#include <boost/thread/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/LinearMath/Vector3.h>
#include "KalmanFilter.h"

namespace helper {
    class TransformationHandler {

    private:
        const double KALMAN_RESET_SQUARED_DISTANCE = 0.0025;
        const int KALMAN_RESET_INTERVALL = 0;

        void Init();

        //ros stuff
        int _iRefreshRate;
        std::string _nodeName = "TransformationHandler";
        ros::NodeHandle *_node;
        boost::thread *_nodeThread;

        void RunThread();

        void Run();

        //tfStuff
        tf2_ros::Buffer _tfBuffer;
        tf2_ros::TransformListener *_tfListener;
        tf2_ros::TransformBroadcaster *_tfBroadcaster;
        tf2_ros::StaticTransformBroadcaster *_tfStaticTransformBroadcaster;

        bool _waitFlag = true;

        //kalman stabilization stuff
        std::map<std::string, KalmanFilter *> stabilizers;
        std::map<std::string, MatrixXd> lastTransforms;

        KalmanFilter *InitializeKalmanFilter(geometry_msgs::Transform firstTransform);

        Eigen::MatrixXd TransformToMat(geometry_msgs::Transform* transform);

        geometry_msgs::Transform MatToTransform(Eigen::MatrixXd matrix);

        void CheckStabilizers(std::string name, geometry_msgs::Transform *rawTransform);

        bool CheckDistanceToLastTF(std::string name, MatrixXd *rawTransform);


    public:
        TransformationHandler(int refreshRate, std::string baseName = "");

        TransformationHandler(ros::NodeHandle *parentNode, int refreshRate);

        TransformationHandler(ros::NodeHandle *parentNode);

        ~TransformationHandler();

        boost::signals2::signal<void()> sigNodeCircle;

        static geometry_msgs::Transform PoseToTransform(geometry_msgs::Pose pose);

        static geometry_msgs::TransformStamped TransformToStamped(geometry_msgs::Transform transform,
                                                                  std::string fromFrame,
                                                                  std::string toFrame);

        static geometry_msgs::TransformStamped PoseToTransformStamped(geometry_msgs::Pose pose, std::string fromFrame,
                                                                      std::string toFrame);

        void WaitOne();

        bool SendTransform(geometry_msgs::TransformStamped transform);

        bool SendTransform(geometry_msgs::Transform transform, std::string fromFrame, std::string toFrame);

        bool SendTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame);

        bool SendTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);

        bool SendStaticTransform(geometry_msgs::TransformStamped transform);

        bool SendStaticTransform(geometry_msgs::Transform transform, std::string fromFrame, std::string toFrame);

        bool SendStaticTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame);

        bool SendStaticTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);

        bool SendStableTransform(geometry_msgs::TransformStamped transform, double error);
        bool SendStableTransform(geometry_msgs::Transform transform, std::string fromFrame, std::string toFrame,
                                 double error);

        bool CheckTransformation(geometry_msgs::TransformStamped transform);

        geometry_msgs::PoseStamped TransformPose(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame);

        geometry_msgs::PoseStamped TransformPose(geometry_msgs::PoseStamped transformPose, std::string toFrame);

        geometry_msgs::PoseStamped TransformPose(geometry_msgs::TransformStamped transform, geometry_msgs::PoseStamped);

        geometry_msgs::PoseStamped TransformPose(geometry_msgs::PoseStamped pose, std::string fromFrame,
                                                 std::string toFrame);

        geometry_msgs::Pose TransformPose(geometry_msgs::TransformStamped transform, geometry_msgs::Pose);

        geometry_msgs::TransformStamped GetTransform(std::string fromFrame, std::string toFrame);

        bool FrameExist(std::string frameName);


    };
}
