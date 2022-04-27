//
// Created by thomas on 14.06.18.
//

#include <tf/LinearMath/Quaternion.h>
#include "TransformationHandler.h"

namespace helper {

    TransformationHandler::TransformationHandler(int refreshRate = 30, std::string baseName) :
            _iRefreshRate(refreshRate) {
        _node = new ros::NodeHandle(baseName + _nodeName);

        Init();
        RunThread();
    }


    TransformationHandler::TransformationHandler(ros::NodeHandle *parentNode, int refreshRate) :
            _iRefreshRate(refreshRate) {
        _node = new ros::NodeHandle(*parentNode, _nodeName);

        Init();
        RunThread();
    }

    TransformationHandler::TransformationHandler(ros::NodeHandle *parentNode) :
            _iRefreshRate(30) {
        _node = new ros::NodeHandle(*parentNode, _nodeName);

        Init();
        RunThread();
    }


    void TransformationHandler::RunThread() {
        //run node
        _nodeThread = new boost::thread(boost::bind(&TransformationHandler::Run, this));
    }

    void TransformationHandler::Run() {
        ros::Rate rate(_iRefreshRate);
        while (_node->ok()) {
            sigNodeCircle();
            ros::spinOnce();
            _waitFlag = true;
            rate.sleep();
        }
    }

    void TransformationHandler::Init() {
        _tfListener = new tf2_ros::TransformListener(_tfBuffer);
        _tfBroadcaster = new tf2_ros::TransformBroadcaster();
        _tfStaticTransformBroadcaster = new tf2_ros::StaticTransformBroadcaster();
    }

    geometry_msgs::Transform TransformationHandler::PoseToTransform(geometry_msgs::Pose pose) {

        //create new transform and set header and frame ids
        geometry_msgs::Transform ret;
        ret.translation.x = pose.position.x;
        ret.translation.y = pose.position.y;
        ret.translation.z = pose.position.z;
        ret.rotation = pose.orientation;

        return ret;
    }

    geometry_msgs::TransformStamped TransformationHandler::TransformToStamped(geometry_msgs::Transform transform,
                                                                              std::string fromFrame,
                                                                              std::string toFrame) {
        geometry_msgs::TransformStamped ret;
        ret.transform = transform;
        ret.header.frame_id = fromFrame;
        ret.child_frame_id = toFrame;
        return ret;
    }

    void TransformationHandler::WaitOne() {
        _waitFlag = false;
        while (!_waitFlag) {};

        return;
    }

    geometry_msgs::TransformStamped TransformationHandler::PoseToTransformStamped(geometry_msgs::Pose pose,
                                                                                  std::string fromFrame,
                                                                                  std::string toFrame) {
        geometry_msgs::Transform transform = PoseToTransform(pose);
        return TransformToStamped(transform, fromFrame, toFrame);
    }

    bool TransformationHandler::SendTransform(geometry_msgs::TransformStamped transform) {
        bool ret = true;

        try {
            _tfBroadcaster->sendTransform(transform);
            //ROS_INFO_STREAM(
            //       "Send new transform from " << transform.header.frame_id << " to " << transform.child_frame_id);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Can't send new transform" << transform.child_frame_id << ": " << ex.what());
            ret = false;
        }
        return ret;
    }

    bool TransformationHandler::SendTransform(geometry_msgs::Transform transform, std::string fromFrame,
                                              std::string toFrame) {
        geometry_msgs::TransformStamped toSend = TransformToStamped(transform, fromFrame, toFrame);
        return SendTransform(toSend);
    }

    bool TransformationHandler::SendTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame) {
        geometry_msgs::TransformStamped toSend = PoseToTransformStamped(transformPose.pose,
                                                                        transformPose.header.frame_id,
                                                                        toFrame);
        return SendTransform(toSend);
    }

    bool TransformationHandler::SendTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame) {
        geometry_msgs::TransformStamped toSend = PoseToTransformStamped(pose, fromFrame, toFrame);
        return SendTransform(toSend);
    }

    bool TransformationHandler::SendStaticTransform(geometry_msgs::TransformStamped transform) {
        bool ret = true;

        try {
            _tfStaticTransformBroadcaster->sendTransform(transform);
            //ROS_INFO_STREAM("Send new static transform from " << transform.header.frame_id
            //                                                 << " to " << transform.child_frame_id);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Can't send new transform" << transform.child_frame_id << ": " << ex.what());
            ret = false;
        }
        return ret;
    }

    bool TransformationHandler::SendStaticTransform(geometry_msgs::Transform transform, std::string fromFrame,
                                                    std::string toFrame) {
        geometry_msgs::TransformStamped toSend = TransformToStamped(transform, fromFrame, toFrame);
        return SendStaticTransform(toSend);
    }

    bool TransformationHandler::SendStaticTransform(geometry_msgs::PoseStamped transformPose, std::string toFrame) {
        geometry_msgs::TransformStamped toSend = PoseToTransformStamped(transformPose.pose,
                                                                        transformPose.header.frame_id,
                                                                        toFrame);
        return SendStaticTransform(toSend);
    }

    bool
    TransformationHandler::SendStaticTransform(geometry_msgs::Pose pose, std::string fromFrame, std::string toFrame) {
        geometry_msgs::TransformStamped toSend = PoseToTransformStamped(pose, fromFrame, toFrame);
        return SendStaticTransform(toSend);
    }

    bool TransformationHandler::CheckTransformation(geometry_msgs::TransformStamped transform)
    {
        return true;
    }

    geometry_msgs::TransformStamped TransformationHandler::GetTransform(std::string fromFrame, std::string toFrame) {
        geometry_msgs::TransformStamped transformStamped;

        try {
            transformStamped = _tfBuffer.lookupTransform(toFrame, fromFrame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Can't get transform from " << fromFrame << " to " << toFrame << " :" << ex.what());
        }

        return transformStamped;
    }

    geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::Pose pose, std::string fromFrame,
                                                                    std::string toFrame) {
        auto transform = GetTransform(fromFrame, toFrame);
        geometry_msgs::PoseStamped newPose;
        newPose.pose = pose;
        tf2::doTransform(newPose, newPose, transform);
        return newPose;

    }

    geometry_msgs::PoseStamped
    TransformationHandler::TransformPose(geometry_msgs::PoseStamped pose, std::string fromFrame,
                                         std::string toFrame) {
        auto transform = GetTransform(fromFrame, toFrame);
        tf2::doTransform(pose, pose, transform);
        pose.header.frame_id = toFrame;
        return pose;

    }

    geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::PoseStamped transformPose,
                                                                    std::string toFrame) {
        auto transform = GetTransform(transformPose.header.frame_id, toFrame);
        geometry_msgs::PoseStamped newPose;
        newPose.pose = transformPose.pose;

        tf2::doTransform(transformPose, newPose, transform);
        return newPose;
    }

    bool TransformationHandler::CheckDistanceToLastTF(std::string name, MatrixXd *rawTransform)
    {
        Eigen::MatrixXd mat1(3,1);
        mat1 << rawTransform->data()[0], rawTransform->data()[1], rawTransform->data()[2];
        Eigen::MatrixXd mat2(3,1);
        mat2 << lastTransforms[name].data()[0], lastTransforms[name].data()[1], lastTransforms[name].data()[2];

        if ((mat1 - mat2).norm() < KALMAN_RESET_SQUARED_DISTANCE)
            return true;
        else
        {
            ROS_INFO_STREAM("Reset Kalman Pose Filter for Transformation " + name);
            return false;
        }
    }

    bool TransformationHandler::SendStableTransform(geometry_msgs::TransformStamped transform, double error = 0.01)
    {
        std::string stabilizerName = transform.header.frame_id + transform.child_frame_id;
        CheckStabilizers(stabilizerName, &transform.transform);
        Eigen::MatrixXd mat = TransformToMat(&transform.transform);
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(7, 7);
        //positioning errors in meter
        R(0,0) = mat(0);
        R(1,1) = mat(1);
        R(2,2) = mat(2);
        //positioning errors in rad
        R(3,3) = mat(3);//M_PI/8;
        R(4,4) = mat(4);//M_PI/8;
        R(5,5) = mat(5);//M_PI/8;
        R(6,6) = mat(6);//M_PI/8;
        R = R * error;

        if(CheckDistanceToLastTF(stabilizerName, &mat))
        {
            mat = stabilizers[stabilizerName]->Update(mat, R);
            transform.transform = MatToTransform(mat);

        } else
        {
            //distance over treashold will reset/reinitialize stabilizer
            stabilizers[stabilizerName] = InitializeKalmanFilter(transform.transform);
        }

        //set last transform to new transform
        lastTransforms[stabilizerName] = mat;

        return SendTransform(transform);
    }

    bool TransformationHandler::SendStableTransform(geometry_msgs::Transform transform, std::string fromFrame,
                                                    std::string toFrame, double error = 0.01)
    {
        geometry_msgs::TransformStamped toSend = TransformToStamped(transform, fromFrame, toFrame);

        return SendStableTransform(toSend, error);
    }

    void TransformationHandler::CheckStabilizers(std::string name, geometry_msgs::Transform *rawTransform)
    {
        //check if stabilizers for transformation is allready known
        if (stabilizers.count(name) <= 0)
        {
            //not known so we need to initioalize our Kalman stabilizers
            KalmanFilter *kalmanFilter = InitializeKalmanFilter(*rawTransform);

            stabilizers[name] = kalmanFilter;
            //init also the last transformation with the current transform
            lastTransforms[name] = TransformToMat(rawTransform);
        }
    }

    geometry_msgs::Transform TransformationHandler::MatToTransform(Eigen::MatrixXd mat) {
        AngleAxisd angleAxis;
        //the values vor the axis and the angle are hidden in the whole 7x1 pose Stabilisation matrix!
        angleAxis.axis() = Vector3d(mat(3, 0), mat(4, 0), mat(5, 0));
        angleAxis.angle() = mat(6, 0);
        Eigen::Quaterniond q(angleAxis);
         q.normalize();

        geometry_msgs::Transform ret;
        ret.translation.x = mat(0, 0);
        ret.translation.y = mat(1, 0);
        ret.translation.z = mat(2, 0);
        ret.rotation.x = q.x();
        ret.rotation.y = q.y();
        ret.rotation.z = q.z();
        ret.rotation.w = q.w();

        return ret;
    }

    Eigen::MatrixXd TransformationHandler::TransformToMat(geometry_msgs::Transform* transform)
    {

        Eigen::Quaterniond q(transform->rotation.w, transform->rotation.x, transform->rotation.y, transform->rotation.z);
        AngleAxisd anglesAxis(q);

        Eigen::MatrixXd ret(7,1);
        ret(0,0) = transform->translation.x;
        ret(1,0) = transform->translation.y;
        ret(2,0) = transform->translation.z;
        ret(3,0) = anglesAxis.axis().data()[0];
        ret(4,0) = anglesAxis.axis().data()[1];
        ret(5,0) = anglesAxis.axis().data()[2];
        ret(6,0) = anglesAxis.angle();

        return ret;
    }


    KalmanFilter *TransformationHandler::InitializeKalmanFilter(geometry_msgs::Transform firstTransform) {
        MatrixXd A;
        A = A.Identity(7,7);
        MatrixXd B(7,7);
        B.setZero();

        KalmanFilter *ret = new KalmanFilter(A, B, KALMAN_RESET_INTERVALL);

        MatrixXd y = TransformToMat(&firstTransform);
        MatrixXd P0 = MatrixXd::Identity(7,7);

        for (int i=0; i<P0.rows(); i++)
        {
            P0(i,i) = y(i, 0) * y(i, 0);
        }

        ret->InitFilter(y, P0);

        return ret;
    }

    geometry_msgs::PoseStamped TransformationHandler::TransformPose(geometry_msgs::TransformStamped transform,
                                                                    geometry_msgs::PoseStamped pose) {
        geometry_msgs::PoseStamped ret;
        tf2::doTransform(pose, ret, transform);
        return ret;
    }

    geometry_msgs::Pose TransformationHandler::TransformPose(geometry_msgs::TransformStamped transform,
                                                             geometry_msgs::Pose pose) {
        geometry_msgs::PoseStamped ret;
        ret.pose = pose;
        tf2::doTransform(pose, ret.pose, transform);
        return ret.pose;
    }


    bool TransformationHandler::FrameExist(std::string frameName) {
        bool ret = false;
        try {
            ret = _tfBuffer._frameExists(frameName);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN_STREAM("Can't check frame existance of " << frameName << ex.what());
        }

        return ret;
    }

    TransformationHandler::~TransformationHandler() {
        delete _tfListener;
        delete _tfBroadcaster;
        delete _tfStaticTransformBroadcaster;
    }
}