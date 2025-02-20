#include "orientation_estimator.h"

#include <cmath>

//Default constructor, input timestep
orientation_estimator::pose_pos_kalman_estimator(float dt) {
	this->dt = dt;
}

//Run the three update equations and set the internal members they are used to calculate using measurement vector (z)
void orientation_estimator::update(Vector3f z_accel) {
	//1: Kalman Gain Calculation
	kalman_gain_M = proj_state_cov_M*observation_M.transpose() * (observation_M*proj_state_cov_M*observation_M.transpose() + measurement_cov_M).inverse();

	//2: State Update Equation
	state_vector = proj_state_vector + kalman_gain_M*(z - observation_M*proj_state_vector);

	//3: Covariance Update Equation
	state_cov_M = (I - kalman_gain_M*observation_M)*proj_state_cov_M*(I - kalman_gain_M*observation_M).transpose() + kalman_gain_M*measurement_cov_M*kalman_gain_M.transpose();

	//0: Convert the accelerometer axes into fixed pitch and roll measurements by comparing axis measurement ratios
}

//Extrapolate and predict the next orientation by integrating the gyro rates over the time step
void orientation_estimator::predict(Vector3f u) {
	//1: State Extrapolation Equation
	proj_state_vector = state_transition_M*state_vector + control_M*u;
	
	//2: Covariance Extrapolation Equation
	proj_state_cov_M = state_transition_M*state_cov_M*state_transition_M.transpose() + process_noise_cov_M;
}

//Set the initial state and state covariance, and project next state using gyro rates
bool orientation_estimator::set_initial_state_and_predict(Quaternionf x, MatrixXf P, Vector3f u_gyro) {
	//Confirm dimensions of vector and matrix
	if (x.size() != n_x || P.rows() != n_x || P.cols() != n_x) {
		return false;
	}

	//With dimensions verified, set internal vector and matrix
	state_vector = x;
	state_cov_M = P;

	//Run the prediction step based on the initial state and covariance
	predict(u_gyro);

	return true;
}

//Helper update method for the state transition matrix, set the actual matrix F from the coefficient and power matrices
void orientation_estimator::dt_update_state_transition_M(float dt) {
	//TODO: Figure out a convenient way to set up the F matrix dt update and store powers and coefficients
	
}

//Setter for state transition matrix (F) - intakes matrix of coefficients of dt components, and matrix of their powers
bool orientation_estimator::set_state_transition_M(MatrixXf dt_coeff_M, MatrixXf dt_pow_M) {
	//TODO: There's definitely some pointer weirdness that needs to be done here, for now working on primarily structure
	
	//Confirm that both matrices are of the correct dimension
	if (dt_coeff_M.cols() != n_x || dt_coeff_M.rows() != n_x || dt_pow_M.cols() != n_x || dt_pow_M.rows() != n_x) {
		return false;
	}
	
	//With dimensions verified, set each of the two internal matrices
	F_dt_coeff_M = dt_coeff_M;
	F_dt_pow_M = dt_pow_M;
	return true;
}

//Setter for the observation matrix (H)
bool orientation_estimator::set_observation_M(MatrixXf H) {
	//Confirm dimensions of matrix
	if (H.rows() != n_z || H.cols() != n_x) {
		return false;
	}

	//With dimensions verified, set internal matrix
	observation_M = H;

	return true;
}

//Setter for the control matrix (B, sometimes G)
bool orientation_estimator::set_control_M(MatrixXf B) {
	//Confirm dimensions of matrix
	if (B.rows() != n_x || B.cols() != n_u) {
		return false;
	}

	//With dimensions verified, set internal matrix
	control_M = B;

	return true;
}

//Setter for the process covariance matrix (Q)
bool orientation_estimator::set_process_cov_M(MatrixXf Q) {
	//Confirm dimensions of matrix
	if (Q.rows() != n_x || Q.cols() != n_x) {
		return false;
	}

	//With dimensions verified, set internal matrix
	process_noise_cov_M = Q;

	return true;
}

//Setter for the measurement covariance matrix (R)
bool kalman_filter::set_measurement_cov_M(MatrixXf R) {
	//Confirm dimensions of matrix
	if (R.rows() != n_z || R.cols() != n_z) {
		return false;
	}

	//With dimensions verified, set internal matrix
	measurement_cov_M = R;

	return true;
}
