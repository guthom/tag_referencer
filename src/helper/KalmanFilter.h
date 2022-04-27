//
// Created by thomas on 07.11.18.
//

#ifndef PROJECT_KALMANFILTER_H
#define PROJECT_KALMANFILTER_H

#include <Eigen/Dense>


using namespace Eigen;
namespace helper {
    class KalmanFilter {

    public:
        KalmanFilter(MatrixXd A, MatrixXd B);

        KalmanFilter(MatrixXd A, MatrixXd B, int resetCounter);

        ~KalmanFilter();

        void SetW(MatrixXd W);
        void SetQ(MatrixXd Q);
        void SetU(MatrixXd U);

        void InitFilter();

        void InitFilter(MatrixXd X0);
        void InitFilter(MatrixXd X0, MatrixXd P0);

        MatrixXd Update(MatrixXd Y, MatrixXd R);

        MatrixXd Update(MatrixXd Y, MatrixXd R, double dT);

        MatrixXd GetCurrentState();

        void CheckShape(MatrixXd mat, long rows, long cols, std::string name);
        void PrintMatrix(MatrixXd mat);
    private:

        /*
         * Pk = state covariance Matrices
         * Qk = process noise covariance Matrices
         * Xk =
         * R  = Measurement covariance Matrix
         *
         * K = Kalman Gain!
         *
         * */
        //Previous States
        MatrixXd Xk_prev, Pk_prev;

        //Current predictive States
        MatrixXd Xk_pred, Pk_pred;

        //Current States
        MatrixXd Xk_current, Pk_current;

        //Transformation Matrices
        MatrixXd A, B, C, H;

        //Variables
        MatrixXd R;

        //Error Matrices
        MatrixXd Wk, Qk;

        //Other Matrices
        MatrixXd Uk;



        int resetCounter = -1;
        long iteration = 1;


        double t = 0.0;

        bool initialized = false;

        void Init();

        MatrixXd K(MatrixXd R);

        void UpdatePreviousStates();

        void CalculatePredictiveStates();
        MatrixXd CalculateObservation(MatrixXd rawY);
        void CalculateCurrentStates(MatrixXd Y, MatrixXd R);

        void CheckReset(MatrixXd Y);

    };
}

#endif //PROJECT_KALMANFILTER_H
