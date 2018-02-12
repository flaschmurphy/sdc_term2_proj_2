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
  
    // initial covariance matrix
    P_ = MatrixXd(5, 5);
    P_ << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
  
    // Process noise standard deviation longitudinal acceleration in m/s^2
    // TODO: find the best value here
    std_a_ = 3;
  
    // Process noise standard deviation yaw acceleration in rad/s^2
    // TODO: find the best value here
    std_yawdd_ = 3;
    
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
    //END OF DO NOT MODIFY
    
    /* Complete the initialization. See ukf.h for other member properties.
       Hint: one or more values initialized above might be wildly off...  */

    is_initialized_ = false;

    //set state dimension
    int n_x_ = 5;

    //set augmented dimension
    int n_aug_ = 7;

    //define spreading parameter
    double lambda_ = 3 - n_aug_;

}


// Default destructor
UKF::~UKF() { }


/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    if (!is_initialized_) {
        /**
         * Initialize the state ekf_.x_ with the first measurement.
         * Create the covariance matrix.
         * Remember: you'll need to convert radar from polar to cartesian coordinates 
         */

        double px;
        double py;
        double vx;
        double vy;

        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            // Convert radar from polar to cartesian coordinates and initialize state
            float rho = meas_package.raw_measurements_[0];
            float phi = meas_package.raw_measurements_[1];
            float rhodot = meas_package.raw_measurements_[2];
            px = rho*cos(phi);
            py = rho*sin(phi);
            vx = rhodot*cos(phi);
            vy = rhodot*sin(phi);
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            // Lidar is already in cartesian, so just initialize state
            px = meas_package.raw_measurements_(0);
            py = meas_package.raw_measurements_(1);
            vx = 0;
            vy = 0;
        }

        // Handle small px, py
        if(fabs(px) < 0.0001) { px = 0.0001; }
        if(fabs(py) < 0.0001) { py = 0.0001; }

        x_ << px, py, sqrt(pow(vx, 2) + pow(vy, 2)), 0, 0;

        previous_t_ = meas_package.timestamp_;
        is_initialized_ = true;

        return;
    }

    double delta_t_ = (meas_package.timestamp_ - previous_t_)/1000000.0;
    Prediction(delta_t_);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    }

    previous_t_ = meas_package.timestamp_;
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /** 
     * TODO: 
     * Complete this function! Estimate the object's location. Modify the state vector, x_. 
     * Predict sigma points, the state, and the state covariance matrix.
     */


    /***********************************************************************************************
    *    Create Augmented Sigma Points
    ***********************************************************************************************/

    //create augmented mean vector
    VectorXd x_aug = VectorXd(7);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd(7, 7);

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);


    //create augmented mean state
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;

    //create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5,5) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;

    //create square root matrix
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++) {
      Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
      Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
    }

    /***********************************************************************************************
    *    Predict Augmented Sigma Points
    ***********************************************************************************************/

    //create matrix with predicted sigma points as columns
    MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

    //predict sigma points
    for (int i = 0; i< 2* n_aug_ + 1; i++) {
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
        } else {
            px_p = p_x + v*delta_t*cos(yaw);
            py_p = p_y + v*delta_t*sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd*delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
        py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
        v_p = v_p + nu_a*delta_t;

        yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
        yawd_p = yawd_p + nu_yawdd*delta_t;

        //write predicted sigma point into right column
        Xsig_pred(0,i) = px_p;
        Xsig_pred(1,i) = py_p;
        Xsig_pred(2,i) = v_p;
        Xsig_pred(3,i) = yaw_p;
        Xsig_pred(4,i) = yawd_p;
    }

    /***********************************************************************************************
    *    Predict mean and covariance
    ***********************************************************************************************/

    //create vector for weights
    VectorXd weights = VectorXd(2*n_aug_+1);

    //create vector for predicted state
    VectorXd x = VectorXd(n_x_);

    //create covariance matrix for prediction
    MatrixXd P = MatrixXd(n_x_, n_x_);

    // set weights
    double weight_0 = lambda_/(lambda_ + n_aug_);
    weights(0) = weight_0;
    for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
        double weight = 0.5/(n_aug_ + lambda_);
        weights(i) = weight;
    }

    //predicted state mean
    x.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        x = x+ weights(i) * Xsig_pred.col(i);
    }

    //predicted state covariance matrix
    P.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //iterate over sigma points
        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
        //angle normalization
        while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        P = P + weights(i) * x_diff * x_diff.transpose() ;
    }
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    * TODO: 
    * Complete this function! Use lidar data to update the belief about the object's
    * position. Modify the state vector, x_, and covariance, P_.
    * You'll also need to calculate the lidar NIS.
    */

    VectorXd z_ = VectorXd(2);
    z_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
    UpdateState(2, false);
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    * TODO:
    * Complete this function! Use radar data to update the belief about the object's
    * position. Modify the state vector, x_, and covariance, P_.
    *
    * You'll also need to calculate the radar NIS.
    */

    /***********************************************************************************************
    *    Predict Radar Sigma Points
    ***********************************************************************************************/

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points

      // extract values for better readibility
      double p_x = Xsig_pred(0,i);
      double p_y = Xsig_pred(1,i);
      double v  = Xsig_pred(2,i);
      double yaw = Xsig_pred(3,i);

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
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        S = S + weights(i) * z_diff * z_diff.transpose();
    }
    //add measurement noise covariance matrix
    MatrixXd R = MatrixXd(n_z,n_z);
    R <<  std_radr*std_radr, 0, 0,
          0, std_radphi*std_radphi, 0,
          0, 0,std_radrd*std_radrd;
    S = S + R;

    // TODO: calculate radar NIS

    /***********************************************************************************************
    *    Update Radar 
    ***********************************************************************************************/

    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x, n_z);

    //calculate cross correlation matrix
    Tc.fill(0.0);
    for (int i = 0; i < 2 * n_aug + 1; i++) {  //2n+1 simga points
 
        //residual
        VectorXd z_diff = Zsig.col(i) - z_pred;
        //angle normalization
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

        // state difference
        VectorXd x_diff = Xsig_pred.col(i) - x;
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
    x = x + K * z_diff;
    P = P - K*S*K.transpose();
}


void UKF::UpdateState(int n_z) {
    // n_z: will be 3 for radar and 2 for lidar
  
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);
  
    //calculate cross correlation matrix
    Tc.fill(0.0);
  
    for (int i = 0; i < n_sig_; i++) {
        VectorXd z_diff = Zsig_.col(i) - z_pred_;
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }
  
    //Kalman gain K;
    MatrixXd K = Tc * S_.inverse();
    VectorXd z_diff = z_ - z_pred_;
  
    if (n_z == 3) {    // i.e. radar
        // Normalize angles
        while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
    }
  
    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K*S_*K.transpose();
  
    // Calculate NIS
    double nis = z_diff.transpose() * S_.inverse() * z_diff;
    cout << nis << endl;

}
