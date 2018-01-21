#include "kalman_filter.h"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    /**
      TODO:
    * predict the state
    */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    /**
      TODO:
    * update the state by using Kalman Filter equations
    */
    VectorXd y = z - H_ * x_;
    UpdateInternal(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    /**
      TODO:
    * update the state by using Extended Kalman Filter equations
    */

    const double CLOSE_TO_ZERO = 1e-8;

    double px = x_(0);
    double py = x_(1);
    double vx = x_(2);
    double vy = x_(3);

    double ro = sqrt(px * px + py * py);

    if(fabs(px) < CLOSE_TO_ZERO){
        cout<<"KalmanFilter::UpdateEKF Warning, px close to zero";
        px = CLOSE_TO_ZERO * (px < 0. ? -1. : 1.);
    }

    double phi = atan2(py, px);

    if(fabs(ro) < CLOSE_TO_ZERO){
        cout<<"KalmanFilter::UpdateEKF Warning, ro close to zero";
        ro = CLOSE_TO_ZERO * (ro < 0. ? -1. : 1.);
    }
    double dro = (px * vx + py * vy)/sqrt(px * px + py * py);

    VectorXd h = VectorXd(3);
    h << ro, phi, dro;

    VectorXd y = z - h;

    y(1) = normalizePhi( y(1));

    UpdateInternal(y);

}

void KalmanFilter::UpdateInternal(const Eigen::VectorXd &y)
{
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    //new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

double KalmanFilter::normalizePhi(double phi)
{
    int k = int(phi/M_PI);

    if(fabs(k) > 0){
        phi = phi - k*M_PI;
    }
    return phi;
}

