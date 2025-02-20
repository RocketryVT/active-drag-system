#include "orientation_estimator.h"

#include <cmath>

//Default constructor, input timestep and initialize internal state transition matrices
orientation_estimator::pose_pos_kalman_estimator(float dt) {
	//Set dt value
	this->dt = dt;
}

//Run the three update equations and set the internal members they are used to calculate using measurement vector (z)
void orientation_estimator::update(Vector3f z_accel, Vector3f z_mag) {
	//0: Calculate the new orientation matrix (H) based on the current quaternion coefficients
	
	//1: Kalman Gain Calculation
	kalman_gain_M = proj_state_cov_M*observation_M.transpose() * (observation_M*proj_state_cov_M*observation_M.transpose() + measurement_cov_M).inverse();

	//2: State Update Equation
	state_vector = proj_state_vector + kalman_gain_M*(z - observation_M*proj_state_vector);

	//3: Covariance Update Equation
	state_cov_M = (I - kalman_gain_M*observation_M)*proj_state_cov_M*(I - kalman_gain_M*observation_M).transpose() + kalman_gain_M*measurement_cov_M*kalman_gain_M.transpose();

	//0: Convert the accelerometer axes into fixed pitch and roll measurements by comparing axis measurement ratios
	
}

//Extrapolate and predict the next orientation by integrating the UNBIASED gyro rates over the time step
void orientation_estimator::predict(Vector3f u_gyro) {
	//TODO: Test in lab to confirm gyro output is in radians/degrees and that math matches
	float w_x = u_gyro(0);
	float w_y = u_gyro(1);
	float w_z = u_gyro(2);

	//1: State Extrapolation Equation via multiplicative rotation quaternion addition
	float gyro_mag = u_gyro.norm();					//Calculate magnitude of rotation rate
	Quaternionf update_q(std::cos(gyro_mag/2.0f), 
						 std::sin(gyro_mag/2.0f)*w_x/gyro_mag,
						 std::sin(gyro_mag/2.0f)*w_y/gyro_mag,
						 std::sin(gyro_mag/2.0f)*w_z/gyro_mag);
	//TODO: Eigen's quaternion coefficient ordering is funky, confirm that constructor and math are in right order
	proj_state_quat = proj_state_quat + proj_state_quat*update_q;
	proj_state_quat.normalize();	//Normalize after operation to make sure new quaternion is a rotation quaternion
	
	//2: Update the state transition matrix (F) based on gyro rates, and calculate P(n+1,n) with it
	state_transition_M << 0, -w_x*dt, -w_y*dt, -w_z*dt,
					   	  w_x*dt, 0, w_z*dt, -w_y*dt,
						  w_y*dt, -w_z*dt, 0, w_x*dt,
						  w_z*dt, w_y*dt, -w_x*dt, 0;
	proj_state_cov_M = state_transition_M*state_cov_M*state_transition_M.transpose() + process_noise_cov_M;
	//TODO: Confirm in testing that the gyro rate skew matrix correctly updates covariance/is stable
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
