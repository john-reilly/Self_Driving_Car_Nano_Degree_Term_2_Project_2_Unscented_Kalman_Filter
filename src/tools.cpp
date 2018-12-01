#include <iostream>
#include "tools.h"

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
    // this is a straight copy from my EKF.cpp project file see link
  // https://github.com/john-reilly/Self_Driving_Car_Nano_Degree_Term_2_Project_1_Extended_Kalman_Filter/blob/master/src/tools.cpp
  
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if(estimations.size() != ground_truth.size()
      || estimations.size() == 0){
      cout << "Invalid estimation or ground_truth data" << endl;
      return rmse;
  }

  //accumulate squared residuals
  for(unsigned int i=0; i < estimations.size(); ++i){

      VectorXd residual = estimations[i] - ground_truth[i];

      //coefficient-wise multiplication
      residual = residual.array()*residual.array();
      rmse += residual;
   }

  //calculate the mean
  rmse = rmse/estimations.size();

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
return rmse;
  
}