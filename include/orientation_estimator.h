#pragma once
#include <Eigen/Dense>
using namespace Eigen;

#define G_ACCEL 9.80665

//Specifically designed estimator class; this is NOT abstractly applicable to other systems currently
class orientation_estimator {
	private:
		//dt value, stored as time difference in seconds (in theory 0.002)
		float dt;

		//Equation values				
		Quaternionf state_quat;			// x (n_x | 1)
		Matrix4f state_cov_M;			// P (n_x | n_x)	
		Quaternionf proj_state_quat;	// x(n+1,n) 
		Matrix4f proj_state_cov_M;		// P(n+1,n)

		MatrixXf state_transition_M;	// F (n_x | n_x)
		MatrixXf kalman_gain_M;			// K (n_x | n_z)
		//MatrixXf control_M;			// B (n_x | n_u)
		MatrixXf observation_M;			// H (n_z | n_x)

		MatrixXf process_noise_cov_M;	// Q (n_x | n_x)
		MatrixXf measurement_cov_M;		// R (n_z | n_z)

		MatrixXf I;						//Identity with n_x dimensions, used for Kalman Gain calculation
		
		//Methods for each of the five major equation calculations
		void update(Vector3f z_accel);	//Kalman Gain, State Update, Covariance Update
		void predict(Vector3f u_gyro);	//State Extrapolation, Covariance Extrapolation

	public:
		//Minimally parameterized constructor, intakes dt
		kalman_filter(float dt);
		
		//Setter values for each of the constant matrices in the filter
		bool set_state_transition_M(MatrixXf dt_coeff_M, MatrixXf dt_pow_M);
		bool set_observation_M(MatrixXf H);
		//bool set_control_M(MatrixXf B);
		bool set_process_cov_M(MatrixXf Q);
		bool set_measurement_cov_M(MatrixXf R);

		//Initializer function for t = 0; intake x(0,0), P(0,0), and u(0), and extrapolates x(1,0) and P(1,0)
		bool set_initial_state_and_predict(VectorXf x, MatrixXf P);

		//Calculation method - run update and predict steps with inputs, return state vector
		VectorXf run(VectorXf z, VectorXf u, float dt);
};
