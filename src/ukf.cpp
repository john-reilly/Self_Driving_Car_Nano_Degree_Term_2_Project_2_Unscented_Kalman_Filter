#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);
  x_ << 0,0,0,0,0; 
  
  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,
		  0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
    	  0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2 ; //I tired 0.2, 2.0, 0.15, 0.3, 3 and 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2; //I tired 0.2, 2.0, 0.15, 0.3, 3 and 30
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
 
  ///* State dimension
 n_x_ = 5 ; // 5 as from  Lesson: 7 Section:21 Assignment 2 before student part

  ///* Augmented state dimension
 n_aug_ = 7 ; //  7  from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
   ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1); //  VectorXd weights = VectorXd(2*n_aug+1); //Lesson: 7 Section: 24 Assignment: 2 

  ///* Sigma point spreading parameter
 lambda_ = 3 - n_aug_ ; //3 - n_aug; //also from Lesson 7 Section 18 Assignment 2 but from before student part begins
  //moving this bit weights_made the biggest differnce so far nearly within rubic spec!
 double weight_0 = lambda_/(lambda_+n_aug_);
 weights_(0) = weight_0;
 for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
  
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  previous_timestamp_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
   
    cout << "Initialising UKF: " << endl;
          
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
     
      float rho = meas_package.raw_measurements_(0) ;
      float theta = meas_package.raw_measurements_(1) ;//theta is called phi_measured in lesson 6
      
      x_(0) = rho*cos(theta);
      x_(1) = rho*sin(theta);  
           
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
     //Initialize state.       
      x_(0) = meas_package.raw_measurements_(0) ; 
      x_(1) = meas_package.raw_measurements_(1) ;
      
    }
    
    previous_timestamp_ = meas_package.timestamp_;

    cout << "Initialising complete" << endl ;
    // done initializing
    is_initialized_ = true;
  
    return;
  }
  // SIMILAR TO EKF as per Q+A video
  delta_t = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);//commented out to see how laser only works
  }
  previous_timestamp_ = meas_package.timestamp_;
  
  
  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  cout << "Beginning Prediciton" << endl ;

  //Lesson 7 Section 18 Assignment 2
  
  //create augmented mean vector 
  VectorXd x_aug = VectorXd(7);//also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);//also from Lesson 7 Section 18 Assignment 2 but from before student part begins
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);//also from Lesson 7 Section 18 Assignment 2
  
  cout << "Line 193" << endl ;
  
  //create augmented mean state
  x_aug.head(5) = x_; //this was x not x_in quiz
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_; //was P not P_ in quiz
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();
  cout << "Line 208" << endl ;

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }
  

  cout << "Line 222" << endl ;
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    //predicted state values
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }
  //  cout << "Line 257" << endl ;
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
//cout << "Line 261" << endl ;
    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
//cout << "Line 266" << endl ;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
//cout << "Line 269" << endl ;
//cout << "Line 270 px_p: " << px_p << endl ;
//cout << "Line 271 i : " << i << endl ; // had som eprobelms here used cout to help
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
//cout << "Line 272" << endl ;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
//cout << "Line 277" << endl ;
//cout << "Line 297" << endl ;  
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
//  cout << "Line 309" << endl ;  

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
  cout << "Line 316" << endl ;  

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
//cout << "Line 321" << endl ;  
    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
  //    cout << "Line 324" << endl ;
    //angle normalization
    cout << x_diff(3)  << endl ;
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;//maybe print out x_diff(3)
//      cout << "Line 327" << endl ; //program was getting very stuck here with way off values lowing down everything for minutes at a time after a few iterations
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
//    cout << "Line 329" << endl ;  

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
//cout << "Line 324" << endl ;  


  
  cout << "End Prediciton" << endl ;
  
  
  
  
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  cout << "Beginning Update Lidar" << endl ;


  //changes for Lidar 2 as in x and y //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;// was int n_z = 3;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
   // double v  = Xsig_pred_(2,i); //not currently used ...from exercise
   // double yaw = Xsig_pred_(3,i); //not currently used ...from exercise

   // double v1 = cos(yaw)*v;// not sure for lidar //unused warning//not used.. from exercise
   // double v2 = sin(yaw)*v;// not sure for lidar //unused warning // not used.... from exercise

    //definately changes for lidar// measurement model 
    Zsig(0,i) = p_x ; // was Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = p_y ; // was Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    //Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //this might need to change? //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;// just normalise once somewhere else in a function....
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  //change radar to laser here
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
          
  S = S + R;

    //create example vector for incoming radar measurement
    
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_; //added 6 dec I think I was just overlaying same data repeatly same as update radar
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  
  /*******************************************************************************
 * Student part begin
 ******************************************************************************/
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
//NEED x_ and P_ here??
  //update state mean and covariance matrix
 x_ = x_ + K * z_diff;// was x = x + K * z_diff;
 P_ = P_ - K*S*K.transpose();//was  P = P - K*S*K.transpose();
  
  
  
 //cout << "End Update Lidar" << endl ; 
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
 // cout << "Beginning Update Radar" << endl ;
 
  // Below code from Lesson: 7 Video : 27 Assignment 2:
  
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;


  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);


  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug+1; i++) {
      z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  //innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;// just normalise once somewhere else in a function....
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
  S = S + R;

  
    //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  z = meas_package.raw_measurements_; //added 6 dec I think I was just overlaying same data repeatly


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, n_z);
  
  /*******************************************************************************
 * Student part begin
 ******************************************************************************/
  
  //calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_; // was x 6 dec
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  //residual
  VectorXd z_diff = z - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;//was x = x + K * z_diff; // NEED to USE X_ and P_ here??
  P_ = P_ - K*S*K.transpose();//was P = P - K*S*K.transpose();
  
  cout << "End Update Radar" << endl ;
  
}


