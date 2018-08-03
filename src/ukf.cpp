#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() { // if this is false, laser measurements will be ignored (except during init)
	use_laser_ = true;

	// if this is false, radar measurements will be ignored (except during init)
	use_radar_ = true;

	// initial state vector
	x_ = VectorXd(5);

	// initial covariance matrix
	P_ = MatrixXd(5, 5);

	// Process noise standard deviation longitudinal acceleration in m/s^2
	std_a_ = 1;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 1;

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

	/**
	 TODO:

	 Complete the initialization. See ukf.h for other member properties.

	 Hint: one or more values initialized above might be wildly off...
	 */

	//init spreading parameter
	lambda_ = 0;

	//set state dimension
	n_x_ = 5;

	//set augmented dimension
	n_aug_ = 7;

	//set vector for weights
	weights_ = VectorXd(2 * n_aug_ + 1);

	// State covariance matrix P will be initialized using some data
	// from the first measurement, in ProcessMeasurement() below.
	time_us_ = 0;

	// NIS value
	nis_lidar_file_.open("NIS_lidar.dat");

	// NIS value
	nis_radar_file_.open("NIS_radar.dat");

	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

	is_initialized_ = false;

	cout << " Done Initialization of UKF " << endl;
}

UKF::~UKF() {
	nis_lidar_file_.close();
	nis_radar_file_.close();
}

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
	if (!is_initialized_) {
		cout << "Initializing UKF." << endl;

		// set state to ones
		x_ << 1, 1, 1, 1, 1;
		P_ << 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

		// initial velocity
		float v = 0;

		if (use_radar_
				&& meas_package.sensor_type_ == MeasurementPackage::RADAR) {

			// measurement vector
			float rho, phi, rho_dot;
			rho = meas_package.raw_measurements_(0);
			phi = meas_package.raw_measurements_(1);
			rho_dot = meas_package.raw_measurements_(2);

			// state vector
			float px = rho * cos(phi);
			float py = rho * sin(phi);
			// init v = 0
			float yaw = rho_dot * cos(phi);
			float yawd = rho_dot * sin(phi);

			// initial state measurement
			// Assume CRTV model so v = 1
			x_ << px, py, v, yaw, yawd;

			// Apply Process noise standard deviation in state covariance
			// Assume CRTV model as velocity sigma v is constant, aka 1.
			float std_radr_squared = std_radr_ * std_radr_;
			P_ << std_radr_squared, 0, 0, 0, 0, 0, std_radr_squared, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, std_radphi_, 0, 0, 0, 0, 0, std_radphi_;

			cout << "ProcessMeasurement: radar" << endl;

		} else if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {

			x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), v, 0, 0;

			// Covariance is similar but we just get an XY position
			P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0, 0, std_laspy_
					* std_laspy_, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;

			cout << "ProcessMeasurement: lidar" << endl;
		}

		is_initialized_ = true;
		time_us_ = meas_package.timestamp_;

		return;

	}

	// time interval for prediction
	float deltat = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	// Predict
	Prediction(deltat);

	// Update
	if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		// Radar updates
		UpdateRadar(meas_package);
	} else if (use_laser_) {
		// Laser updates
		UpdateLidar(meas_package);
	}

//cout << "x_ = " << x_ << endl;
//cout << "P_ = " << P_ << endl;
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
	/**
	 TODO:

	 Complete this function! Estimate the object's location. Modify the state
	 vector, x_. Predict sigma points, the state, and the state covariance matrix.
	 */

	/////////////////////////////////////////////////////////////////////////////
	// See Lesson 13
	//
	// handle sigma points
	//

	// define spreading parameter
	lambda_ = 3 - n_x_;

	// Create sigma points
	MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
	MatrixXd A = P_.llt().matrixL();
	Xsig.col(0) = x_;
	for (int i = 0; i < n_x_; i++) {
		Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
		Xsig.col(i + 1 + n_x_) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
	}

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 18
    //
	// Augment sigma points

	//define spreading parameter
	lambda_ = 3 - n_aug_;

	//create augmented mean state
	VectorXd x_aug = VectorXd(7);
	x_aug.head(5) = x_;
	x_aug(5) = 0;
	x_aug(6) = 0;

	//create augmented covariance matrix
	MatrixXd P_aug = MatrixXd(7, 7);
	P_aug.fill(0.0);
	P_aug.topLeftCorner(5, 5) = P_;
	P_aug(5, 5) = std_a_ * std_a_;
	P_aug(6, 6) = std_yawdd_ * std_yawdd_;

	//create square root matrix
	MatrixXd L = P_aug.llt().matrixL();

	//create augmented sigma points
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; i++) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
		Xsig_aug.col(i + 1 + n_aug_) = x_aug
				- sqrt(lambda_ + n_aug_) * L.col(i);
	}

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 21
    //
	// predict sigma points
	//
	// TODO: may need lesson 27 for RADAR vs LASER

	// Convert coordinate systems
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		//extract values for better readability
		double p_x = Xsig_aug(0, i);
		double p_y = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawd = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);

		//predicted state values
		double px_p, py_p;

		//avoid division by zero
		if (fabs(yawd) > 0.001) {
			px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
			py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
		} else {
			px_p = p_x + v * delta_t * cos(yaw);
			py_p = p_y + v * delta_t * sin(yaw);
		}

		// Put in same ref frame as lidar
		double v_p = v;
		double yaw_p = yaw + yawd * delta_t;
		double yawd_p = yawd;

		//add noise
		px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
		py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
		v_p = v_p + nu_a * delta_t;
		yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
		yawd_p = yawd_p + nu_yawdd * delta_t;

		//write predicted sigma point into right column
		Xsig_pred_(0, i) = px_p;
		Xsig_pred_(1, i) = py_p;
		Xsig_pred_(2, i) = v_p;
		Xsig_pred_(3, i) = yaw_p;
		Xsig_pred_(4, i) = yawd_p;
	}

	// Initialize weights
	double weight_0 = lambda_ / (lambda_ + n_aug_);
	weights_(0) = weight_0;

	// calc weights
	for (int i = 1; i < 2 * n_aug_ + 1; i++) {
		double weight = 0.5 / (n_aug_ + lambda_);
		weights_(i) = weight;
	}

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 23
    //
	// Predict state

	//create vector for predicted state
	x_.fill(0.0);

	//create covariance matrix for prediction
	P_.fill(0.0);

	// Perform PREDICTION
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		x_ = x_ + weights_(i) * Xsig_pred_.col(i);
	}

	// Update COVARIANCE MATRIX
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
	}
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Use lidar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the lidar NIS.
	 */

	//
	int n_z = 2;  // LASER IS XY, aka 2D

	// Setup values to get prediction
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	VectorXd z_pred = VectorXd(n_z);


	/////////////////////////////////////////////////////////////////////////////
	// See lesson 26
    //
	// mean predicted measurement

	Zsig.fill(0.0);
	z_pred.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Zsig(0, i) = Xsig_pred_(0, i);
		Zsig(1, i) = Xsig_pred_(1, i);
	}

	// Get translated prediction
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 26
    //
	// innovation covariance matrix S

	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	// add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << (std_laspx_ * std_laspx_), 0, 0, (std_laspy_ * std_laspy_);

	S = S + R;

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 30
    //

	//calculate cross correlation matrix

	// init Z
	VectorXd z = meas_package.raw_measurements_;

	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sig points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;

		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		//angle normalization
		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	while (z_diff(1) > M_PI)
		z_diff(1) -= 2. * M_PI;
	while (z_diff(1) < -M_PI)
		z_diff(1) += 2. * M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	// NIS part!
	nis_lidar_file_ << z_diff.transpose() * S.inverse() * z_diff;
	nis_lidar_file_ << "\n";
	nis_lidar_file_.flush();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	 TODO:

	 Complete this function! Use radar data to update the belief about the object's
	 position. Modify the state vector, x_, and covariance, P_.

	 You'll also need to calculate the radar NIS.
	 */
	int n_z = 3;  // LASER IS XY
	MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	VectorXd z_pred = VectorXd(n_z);

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 26
    //
	// mean predicted measurement

	Zsig.fill(0.0);
	z_pred.fill(0.0);

	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		// extract values for better readibility
		double p_x = Xsig_pred_(0, i);
		double p_y = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);

		double v1 = cos(yaw) * v;
		double v2 = sin(yaw) * v;

		// measurement model
		Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                        //r
		Zsig(1, i) = atan2(p_y, p_x);                                 //phi
		Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y); //r_dot

	}

	// Get translated prediction
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred = z_pred + weights_(i) * Zsig.col(i);
	}

	/////////////////////////////////////////////////////////////////////////////
	// See lesson 26
    //
	// innovation covariance matrix S

	MatrixXd S = MatrixXd(n_z, n_z);
	S.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 simga points
		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;

		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		S = S + weights_(i) * z_diff * z_diff.transpose();
	}

	//add measurement noise covariance matrix
	MatrixXd R = MatrixXd(n_z, n_z);
	R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_
			* std_radrd_;
	S = S + R;

	/////////////////////////////////////////////////////////////////////////////
	// Lesson 30
	// calculate cross correlation matrix

	// init Z
	VectorXd z = meas_package.raw_measurements_;

	MatrixXd Tc = MatrixXd(n_x_, n_z);
	Tc.fill(0.0);
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {  //2n+1 sig points

		//residual
		VectorXd z_diff = Zsig.col(i) - z_pred;
		//angle normalization
		while (z_diff(1) > M_PI)
			z_diff(1) -= 2. * M_PI;
		while (z_diff(1) < -M_PI)
			z_diff(1) += 2. * M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		//angle normalization
		while (x_diff(3) > M_PI)
			x_diff(3) -= 2. * M_PI;
		while (x_diff(3) < -M_PI)
			x_diff(3) += 2. * M_PI;

		Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
	}

	//Kalman gain K;
	MatrixXd K = Tc * S.inverse();

	//residual
	VectorXd z_diff = z - z_pred;

	while (z_diff(1) > M_PI)
		z_diff(1) -= 2. * M_PI;
	while (z_diff(1) < -M_PI)
		z_diff(1) += 2. * M_PI;

	//update state mean and covariance matrix
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	// NIS part!
	double tmp = (z_diff.transpose() * S.inverse() * z_diff);
	nis_radar_file_ << tmp;
	nis_radar_file_ << "\n";
	nis_radar_file_.flush();

}
