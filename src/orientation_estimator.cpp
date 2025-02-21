#include "orientation_estimator.h"

#include <cmath>

using std::sin, std::cos, std::atan, std::atan2, std::pow, std::sqrt;

//Default constructor, input timestep and initialize internal state transition matrices
orientation_estimator::pose_pos_kalman_estimator(float dt) {
	//Set dt value
	this->dt = dt;
}

//Run the three update equations and set the internal members they are used to calculate using measurement vector (z)
void orientation_estimator::update(Vector3f z_accel, Vector3f z_mag) {
	//0: Convert z_accel measurements into pitch/roll, and convert mag into earth frame and *then* into yaw
	float a_xr = z_accel(0);
	float a_yr = z_accel(1);
	float a_zr = z_accel(2);
	float m_xr = z_mag(0);
	float m_yr = z_mag(1);
	float m_zr = z_mag(2);
	//TODO: I know std::pow() is inefficient; I just don't know what would be more efficient to replace it with
	//TODO: Confirm in testing whether quaternion conversion expects radian or degree input
	float pitch = atan(a_yr/(sqrt(pow(a_xr, 2) + pow(a_zr, 2))));	//Radians
	float roll =  atan(a_xr/(sqrt(pow(a_yr, 2) + pow(a_zr, 2))));	//Radians
	float yaw =   atan2(-m_yr*cos(roll) + m_zr*sin(roll), 
						m_xr*cos(pitch) + m_yr*sin(roll*sin(pitch)+m_zr*cos(roll)*sin(pitch)));
		
	//1: Kalman Gain Calculation
	Quaternionf z_quat = euler2quat(yaw, pitch, roll);
	kalman_gain_M = proj_state_cov_M*observation_M.transpose() * (observation_M*proj_state_cov_M*observation_M.transpose() + measurement_cov_M).inverse();

	//2: State Update Equation
	state_vector = proj_state_vector + kalman_gain_M*(z - observation_M*proj_state_vector);

	//3: Covariance Update Equation
	state_cov_M = (I - kalman_gain_M*observation_M)*proj_state_cov_M*(I - kalman_gain_M*observation_M).transpose() + kalman_gain_M*measurement_cov_M*kalman_gain_M.transpose();

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

//Helper method, convert 3 euler angles to equivalent rotation quaternion - directly from Wikipedia
Quaternionf euler2quat(float yaw, float pitch, float roll) {
	float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    Quaternion q{
    	cr * cp * cy + sr * sp * sy,
   		sr * cp * cy - cr * sp * sy,
    	cr * sp * cy + sr * cp * sy,
    	cr * cp * sy - sr * sp * cy
	};
	
	return q;
}

//Helper method, convert quaternion to vector of equivalent euler angles (yaw, pitch, roll) - directly from Wikipedia
Vector3f quat2euler(Quaternionf q) {
	// Roll
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch
    float sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    float cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    float pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // Yaw 
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    return {yaw, pitch, roll};
}
