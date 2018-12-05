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
  x_ << 0,0,0,0,0; // added 4 dec I thin itialising in funcitons below is causing reeated intis

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1, 0, 0, 0, 0,// added 4 dec I thin itialising in funcitons below is causing reeated intis
		  0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
    	  0, 0, 0, 0, 1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;//Q+A says change this

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;//Q+A says change this
  
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
  //from header variables file adding here as indicated by header ukf.h
  

  ///* State dimension
 n_x_ = 5 ; // 5 as from  Lesson: 7 Section:21 Assignment 2 before student part

  ///* Augmented state dimension
 n_aug_ = 7 ; //  7  from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
   ///* Weights of sigma points
  weights_ = VectorXd(2*n_aug_+1); //  VectorXd weights = VectorXd(2*n_aug+1); //Lesson: 7 Section: 24 Assignment: 2 

  ///* Sigma point spreading parameter
 lambda_ = 3 - n_aug_ ; //3 - n_aug; //also from Lesson 7 Section 18 Assignment 2 but from before student part begins
  
  //delta time
// delta_t; //maybe not needed to init here??
  
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);//MatrixXd Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);//moved this from line 209 363 and 567 yes had it 3 times!
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  previous_timestamp_ = 0;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  //as suggested by Q+A video use EKF initialisation
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
    // initial covariance matrix
  //  P_ << 1, 0, 0, 0, 0,//might change this I am manually putting in identity matirx for now
//		  0, 1, 0, 0, 0,
//		  0, 0, 1, 0, 0,
//		  0, 0, 0, 1, 0,
//    	  0, 0, 0, 0, 1;
    
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
    UpdateRadar(meas_package);
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
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  
  //Create Augmented Sigma Points as per Q+A video
  //Lesson 7 Section 18 Assignment 2
  
  /*******************************************************************************
 * Student part begin, Lesson 7 Section 18 Assignment 2
 ******************************************************************************/
  //create augmented mean vector 
  VectorXd x_aug = VectorXd(7);//also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);//also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
  //Process noise standard deviation longitudinal acceleration in m/s^2
  //double std_a = 0.2; //also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
  //Process noise standard deviation yaw acceleration in rad/s^2
 // double std_yawdd = 0.2;//also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  //set augmented dimension
  //int n_aug = 7;//also from Lesson 7 Section 18 Assignment 2 but from beofre student part begins
  
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);//also from Lesson 7 Section 18 Assignment 2 but from before student part begins
  //define spreading parameter
  //now in constructo
  //double lambda = 3 - n_aug; //also from Lesson 7 Section 18 Assignment 2 but from before student part begins
  
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
  
/*******************************************************************************
 * Student part end, Lesson 7 Section 18 Assignment 2
 ******************************************************************************/
  
  //as per Q+A Lesson: 7 Section:21 Assignment 2
  /*******************************************************************************
 * Student part begin //Lesson: 7 Section:21 Assignment 2
 ******************************************************************************/
  //set state dimension
  //now in constructor
  //int n_x = 5; // from Student part begin //Lesson: 7 Section:21 Assignment 2 before student part
  
  //create matrix with predicted sigma points as columns
//  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);// from Student part begin //Lesson: 7 Section:21 Assignment 2 before student part
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
    cout << "Line 257" << endl ;
    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;
cout << "Line 261" << endl ;
    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;
cout << "Line 266" << endl ;
    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
cout << "Line 269" << endl ;
cout << "Line 270 px_p: " << px_p << endl ;
cout << "Line 271 i : " << i << endl ;
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
cout << "Line 272" << endl ;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
cout << "Line 277" << endl ;
/*******************************************************************************
 * Student part end //Lesson: 7 Section:21 Assignment 2
 ******************************************************************************/
  //as Per Q+A predict mean and covariance
  //Lesson: 7 Section: 24 Assignment: 2
  
  /*******************************************************************************
 * Student part begin  //Lesson: 7 Section: 24 Assignment: 2
 ******************************************************************************/
  //create vector for weights
  VectorXd weights = VectorXd(2*n_aug_+1); //Lesson: 7 Section: 24 Assignment: 2 before student part
  
  //create vector for predicted state
  //now in constructor
  //VectorXd x = VectorXd(n_x);//Lesson: 7 Section: 24 Assignment: 2 before student part
  
  //create covariance matrix for prediction
  //now in constructor
  //MatrixXd P = MatrixXd(n_x, n_x);//Lesson: 7 Section: 24 Assignment: 2 before student part
cout << "Line 297" << endl ;  
  // set weights
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
    x_ = x_+ weights(i) * Xsig_pred_.col(i);
  }

  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose() ;
  }
cout << "Line 324" << endl ;  

/*******************************************************************************
 * Student part end  //Lesson: 7 Section: 24 Assignment: 2
 ******************************************************************************/
  
  cout << "End Prediciton" << endl ;
  
  
  
  
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  cout << "Beginning Update Lidar" << endl ;
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  // I am beginning by taking the update radar code and editing from there
    // Below code from Lesson: 7 Video : 27 Assignment 2:
  
  //set state dimension
  //now in constructor
 // int n_x = 5; //stays the same I think?

  //set augmented dimension
  //now in constructor
  //int n_aug = 7;

  //changes for Lidar 2 as in x and y //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 2;// was int n_z = 3;

  //define spreading parameter
  //now in constructor
  //double lambda = 3 - n_aug;//same as radar?

  //set vector for weights //same as radar?
  VectorXd weights = VectorXd(2*n_aug_+1);
   double weight_0 = lambda_/(lambda_+n_aug_);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug_+1; i++) {  
    double weight = 0.5/(n_aug_+lambda_);
    weights(i) = weight;
  }

  // I note the next 3 variables had std_laser evivalents at top of file....
  //radar measurement noise standard deviation radius in m
 // double std_radr = 0.3; //now in constructor

  //radar measurement noise standard deviation angle in rad
  //double std_radphi = 0.0175; //now in constructor

  //radar measurement noise standard deviation radius change in m/s
  //double std_radrd = 0.1; //now in constructor
  // filling Xsig_pred also seems suspect.....
  //create example matrix with predicted sigma points
  //MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  //commenting out 4 dec
  //Xsig_pred <<    
//    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
//                1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
//          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
//         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
//          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  // I think this is OK to stay same...
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

  //transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;// not sure for lidar //unused warning
    double v2 = sin(yaw)*v;// not sure for lidar //unused warning

    //definately changes for lidar// measurement model 
    Zsig(0,i) = p_x ; // was Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                        //r
    Zsig(1,i) = p_y ; // was Zsig(1,i) = atan2(p_y,p_x);                                 //phi
    //Zsig(2,i) = (p_x*v1 + p_y*v2 ) / sqrt(p_x*p_x + p_y*p_y);   //r_dot
  }

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i=0; i < 2*n_aug_+1; i++) {
      z_pred = z_pred + weights(i) * Zsig.col(i);
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

    S = S + weights(i) * z_diff * z_diff.transpose();
  }
  //change radar to laser here
  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  //R <<    std_radr*std_radr, 0, 0,
  //        0, std_radphi*std_radphi, 0,
  //        0, 0,std_radrd*std_radrd;
  
  R <<    std_laspx_*std_laspx_, 0,
          0, std_laspy_*std_laspy_;
          
  S = S + R;

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/
  // This is Lesson 7 Video 30 Assignemnt 2 as per Q+A
 //now update lidar  //UPDATE RADAR
  
  //maybe don't need this for lidar
  //tohers leave this out eniterly until next section
  // NOTE x and p below are same as x_ and P_ above in class variables and should be used instead??
  //create example vector for predicted state mean
  // Don't need these initalisations ans x replaces x_ not needed and P replacing P_
  
 // VectorXd x = VectorXd(n_x); //now in constructor
  /*
  x <<
     5.93637,
     1.49035,
     2.20528,
    0.536853,
    0.353577; //not needed ??
  //PRIME SUSPECT ...am I setting z each time below?? same in LIDAR??
    //create example vector for incoming radar measurement
    */
  VectorXd z = VectorXd(n_z);
  /*
  z <<
      5.9214,
      0.2187,
      2.0062; //not needed ??
      */
  
    //create example matrix for predicted state covariance
  //MatrixXd P = MatrixXd(n_x,n_x); //now in constructor
  /*
  P <<
  0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972; // not needed?
*/
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

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
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
  
  
  
  
 cout << "End Update Lidar" << endl ; 
  
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  cout << "Beginning Update Radar" << endl ;
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  
  // Below code from Lesson: 7 Video : 27 Assignment 2:
  
  //set state dimension
  int n_x = 5;

  //set augmented dimension
  int n_aug = 7;

  //set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  //define spreading parameter
  double lambda = 3 - n_aug;

  //set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
   double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for (int i=1; i<2*n_aug+1; i++) {  
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }

  //radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  //radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  //radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

 
  //create example matrix with predicted sigma points
 // MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  //not needed this initialisation below?
  /*
  Xsig_pred <<    
    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
                1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

*/

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

/*******************************************************************************
 * Student part begin
 ******************************************************************************/

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
      z_pred = z_pred + weights(i) * Zsig.col(i);
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

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  //add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R <<    std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
  S = S + R;

  
/*******************************************************************************
 * Student part end
 ******************************************************************************/
  // This is Lesson 7 Video 30 Assignemnt 2 as per Q+A
  //UPDATE RADAR
  
  //create example vector for predicted state mean
  VectorXd x = VectorXd(n_x);
//commenting out 4 dec
  //x <<
//     5.93637,
//     1.49035,
//     2.20528,
//    0.536853,
//    0.353577; //not needed ??
  
    //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
//  z <<
//      5.9214,
//      0.2187,
//      2.0062; //not needed ?? //PRIME SUSPECT am I overwritng each time?? I think the quiz was for one case and that is the problem??
  
    //create example matrix for predicted state covariance
  MatrixXd P = MatrixXd(n_x,n_x);
 //commenting out 4 dec
  //P <<
  //0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  //-0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  //0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
 //-0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
 //-0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972; // not needed?

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
    VectorXd x_diff = Xsig_pred_.col(i) - x;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
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

//note to self x_sig _oredict declared 3 times in three differnt scopes and that is a big problem rewriting the default data over and over fix tomorrow

//note to self 4 dec forgot to remove initalisaiton in radar 
