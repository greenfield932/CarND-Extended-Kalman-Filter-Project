#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check inputs validity:
    //  1. the estimation vector size should not be zero
    //  2. the estimation vector size should equal ground truth vector size
    if(estimations.size() == 0 || ground_truth.size() == 0 ||
            estimations.size() != ground_truth.size())
    {
        cout<<"Tools::CalculateRMSE Error: vector size mismatch"<<endl;
        return VectorXd();
    }

    //accumulate squared residuals
    for(unsigned int i=0; i < estimations.size(); ++i){

        //check input vectors contain 4 values (px, py, vx, vy)
        if(estimations[i].size() != ground_truth[i].size() ||
                    rmse.size() != estimations[i].size())
        {
            cout<<"Tools::CalculateRMSE Error: vector components size mismatch"<<endl;
            return VectorXd();
        }
        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse/estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
    if(x_state.size() != 4){
        cout<<"Tools::CalculateJacobian Erorr: input size unexpected"<<endl;
        return MatrixXd();
    }

    //recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    //check division by zero
    if( fabs(px*px + py*py) < 1e-8 ){
        cout<<"Error: division by zero:"<<px<<py;
        return Hj;
    }
    float px2py2 = px*px + py*py;
    float px2py2_root = sqrt(px2py2);

    //compute the Jacobian matrix
    Hj << px/px2py2_root, py/px2py2_root, 0, 0,
       -py/px2py2, px/px2py2, 0, 0,
       py*(vx*py - vy*px)/pow(px2py2, 3./2.), px*(vy*px - vx*py)/pow(px2py2, 3./2.),
       px/px2py2_root, py/px2py2_root;

    return Hj;
}
