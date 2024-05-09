#include <climits>
#include "../include/kalmanfilter.hpp"

// Private----------------------------------------------------------------------
void KalmanFilter::matrixInit() {

    state_vector.setZero(n);
    state_covariance.setZero(n, n); 
    state_transition_M = MatrixXf::Zero(n, n);  
    control_input_M = MatrixXf::Zero(n, p);
    I = MatrixXf::Identity(n, n);
    measurement_M.setIdentity(m, n); // Setup Measurement Matrix
    process_noise_covariance = MatrixXf::Zero(n, n); 
    measurement_covariance = MatrixXf::Zero(m, m); 

    // Setup State Transition Matrix
    state_transition_M << 1.0, dt, 
        0.0, 1.0;
    
    // Setup Control Input Matrix
    control_input_M << 0.5 * std::pow(dt, 2), // (Linear Displacement Eq.)
        dt; 
    
    // Setup Process-Noise Covariance
    process_noise_covariance(0,0) = 0.01;
    process_noise_covariance(1,1) = 0.1;

    // Setup Measurement Covariance
    measurement_covariance << 1e-12;
}


void KalmanFilter::updateMatrices() {

    state_transition_M(0, 1) = dt;
    control_input_M(0, 0) = 0.5 * std::pow(dt, 2);
    control_input_M(1, 0) = dt;
}


void KalmanFilter::prediction(VectorXf control_vec) {

    state_vector = (state_transition_M * state_vector) + (control_input_M * control_vec);
    state_covariance = (state_transition_M * (state_covariance * state_transition_M.transpose())) + process_noise_covariance;
}

void KalmanFilter::update(VectorXf measurement) {

    // Innovation
    VectorXf y = measurement - (measurement_M * state_vector);

    // Residual/Innovation Covariance
    MatrixXf S = (measurement_M * (state_covariance * measurement_M.transpose())) + measurement_covariance;

    // Kalman Gain
    MatrixXf K = (state_covariance * measurement_M.transpose()) * S.inverse();

    // Update
    state_vector = state_vector + (K * y);
    state_covariance = (I - (K * measurement_M)) * state_covariance;
}



// Public----------------------------------------------------------------------
KalmanFilter::KalmanFilter() {
    
}


KalmanFilter::KalmanFilter(int state_dim, int control_dim, int measurement_dim, double dt) 
    : n(state_dim), p(control_dim), m(measurement_dim), dt(dt) {

    matrixInit();
}

bool KalmanFilter::setInitialState(VectorXf state_vec, MatrixXf state_cov) {

    if (state_vec.size() != n || state_cov.rows() != n) {
        std::cout << "Error: Max State & Covariance Dimension should be " << n << std::endl;
        return false;
    }

    state_vector = state_vec;
    state_covariance = state_cov;
    return true;
}




VectorXf KalmanFilter::run(VectorXf control, VectorXf measurement, double _dt) {
    
     if (control.size() != p || measurement.size() != m) {
        std::cout << "Error: Control Vector Size should be "<< p 
            << " Measurement Vector Size should be " << m << std::endl;
        return state_vector;
    }

    dt = _dt;
    updateMatrices();

    prediction(control);
    update(measurement);

    return state_vector;
}