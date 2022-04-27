//
// Created by thomas on 07.11.18.
//

#include "KalmanFilter.h"
#include <ros/ros.h>
namespace helper {

    using namespace Eigen;

    KalmanFilter::KalmanFilter(MatrixXd A, MatrixXd B) : A(A), B(B){
        Init();

        CheckShape(B, A.rows(), A.rows(), "B");
    }

    KalmanFilter::KalmanFilter(MatrixXd A, MatrixXd B, int resetCounter) : KalmanFilter(A, B)
    {
        this->resetCounter = resetCounter;
    }

    void KalmanFilter::Init()
    {
        using namespace Eigen;
        Xk_prev = MatrixXd::Zero(A.rows(), 1);
        Pk_prev = MatrixXd::Zero(A.rows(), A.cols());

        Xk_pred = MatrixXd::Zero(A.rows(), 1);
        Pk_pred = MatrixXd::Ones(A.rows(), A.cols());

        Xk_current = MatrixXd::Zero(A.rows(), 1);
        Pk_current = MatrixXd::Zero(A.rows(), A.cols());

        Wk = MatrixXd::Zero(A.rows(), 1);
        Qk = MatrixXd::Zero(A.rows(), A.cols());

        C = MatrixXd::Identity(A.rows(), A.cols());
        H = MatrixXd::Identity(A.rows(), A.cols());
        Uk = MatrixXd::Zero(A.rows(), 1);
    }


    void KalmanFilter::CheckShape(MatrixXd mat, long rows, long cols, std::string name="")
    {
        if (mat.rows() != rows || mat.cols() != cols)
            throw std::runtime_error("Matrix " + name +" has wrong shape! Current Shape = (" + std::to_string(mat.rows()) +
                                     ", " + std::to_string(mat.cols()) + ") but should have: (" +
                                     std::to_string(rows) +", " + std::to_string(cols));
    }

    void KalmanFilter::SetW(MatrixXd W)
    {
        CheckShape(W, Wk.rows(), Wk.cols(), "W");
        this->Wk = W;
    }

    void KalmanFilter::SetQ(MatrixXd Q)
    {
        CheckShape(Q, Qk.rows(), Qk.cols(), "Q");
        this->Qk = Q;
    }

    void KalmanFilter::SetU(MatrixXd U)
    {
        CheckShape(U, Uk.rows(), Uk.cols(), "U");
        this->Uk = U;
    }

    void KalmanFilter::InitFilter() {
        Xk_current.setZero();
        Pk_current.setOnes();

        initialized = true;
    }

    void KalmanFilter::InitFilter(MatrixXd X0) {
        //set the initial values as current predictions. UpdatePreviousStates() will set it as prev values in updates
        Xk_current = X0;
        Pk_current = Eigen::MatrixXd::Identity(A.rows(), A.cols());

        initialized = true;
    }


    void KalmanFilter::InitFilter(MatrixXd X0, MatrixXd P0) {
        //set the initial values as current predictions. UpdatePreviousStates() will set it as prev values in updates
        Xk_current = X0;
        Pk_current = P0;

        initialized = true;
    }

    void KalmanFilter::CheckReset(MatrixXd Y)
    {
        if(resetCounter > 0 && iteration > resetCounter)
        {
            InitFilter(Y);
            iteration = 1;
        }
        else
        {
            iteration++;
        }
    }

    MatrixXd KalmanFilter::Update(MatrixXd Y, MatrixXd R) {
        if (!initialized)
            throw std::runtime_error("Filter is not initialized!");

        CheckReset(Y);
        UpdatePreviousStates();
        t += 1.0f;
        CalculatePredictiveStates();
        CalculateCurrentStates(Y, R);

        //PrintMatrix(Xk_current);
        return Xk_current;
    }



    MatrixXd KalmanFilter::Update(MatrixXd Y, MatrixXd R, double dT) {
        if (!initialized)
            throw std::runtime_error("Filter is not initialized!");

        CheckReset(Y);
        UpdatePreviousStates();
        t += dT;
        CalculatePredictiveStates();
        CalculateCurrentStates(Y, R);
        return Xk_current;
    }


    MatrixXd KalmanFilter::CalculateObservation(MatrixXd rawY)
    {
        return C*rawY;
    }

    MatrixXd KalmanFilter::GetCurrentState()
    {
        return this->Xk_current;
    }

    MatrixXd KalmanFilter::K(MatrixXd R)
    {
        MatrixXd H_trans = H.transpose();
        MatrixXd a = (Pk_pred * H_trans);
        MatrixXd b = ((H * Pk_pred * H_trans) + R);

        if(b.determinant() != 0)
            return a * b.inverse();
        else
        {
            //return identy because we cant perform the matrix devision
            return MatrixXd::Identity(R.rows(), R.cols());
        }
    }

    void KalmanFilter::UpdatePreviousStates()
    {
        Xk_prev = Xk_current;
        Pk_prev = Pk_current;
    }

    void KalmanFilter::CalculatePredictiveStates()
    {
        Xk_pred = A * Xk_prev + B * Uk + Wk;
        Pk_pred = A * Pk_prev * A.transpose() + Qk;
    }

    void KalmanFilter::CalculateCurrentStates(MatrixXd Y, MatrixXd R)
    {
        Y = CalculateObservation(Y);
        MatrixXd K = this->K(R);
        Xk_current = Xk_pred + K * (Y - H * Xk_pred);
        Pk_current = (MatrixXd::Identity(Pk_pred.rows(), Pk_pred.cols()) - K * H) * Pk_pred;

        if (Pk_current.determinant() == 0)
        {
            //PrintMatrix(K);
            //PrintMatrix(Xk_current);
            //PrintMatrix(Pk_pred);
        }
    }


    void KalmanFilter::PrintMatrix(MatrixXd mat)
    {
        Eigen::IOFormat format;
        if (mat.cols() > 1)
            format = Eigen::IOFormat(StreamPrecision, 0, ", ", ";\n", "", "", "[\n ", "]");
        else
            format = Eigen::IOFormat(StreamPrecision, 0, ", ", ";  ", "", "", "[", "]");

        ROS_INFO_STREAM(mat.format(format));
    }

    KalmanFilter::~KalmanFilter() {

    }
}