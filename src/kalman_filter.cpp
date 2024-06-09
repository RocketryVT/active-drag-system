#include "kalman_filter.hpp"

void kalman_filter::matrix_initialize() {
    state_vector.setZero(n);
    state_covariance.setZero(n, n); 
    state_transition_M = MatrixXf::Zero(n, n);  
    control_input_M = MatrixXf::Zero(n, p);
    I = MatrixXf::Identity(n, n);
    measurement_M.setIdentity(m, n); // Setup Measurement Matrix
    process_noise_covariance = MatrixXf::Zero(n, n); 
    measurement_covariance = MatrixXf::Zero(m, m); 

    // Setup State Transition Matrix
    state_transition_M << 1.0, dt, 0.0, 1.0;

    // Setup Control Input Matrix
    control_input_M << 0.5 * dt * dt, dt; // (Linear Displacement Eq.)

    // Setup Process-Noise Covariance
    process_noise_covariance(0,0) = 0.01;
    process_noise_covariance(1,1) = 0.1;

    // Setup Measurement Covariance
    measurement_covariance << 1e-12;
}

void kalman_filter::matrix_update() {
    state_transition_M(0, 1) = dt;
    control_input_M(0, 0) = 0.5f * dt * dt;
    control_input_M(1, 0) = dt;
}

void kalman_filter::predict(VectorXf control_vec) {
    state_vector = (state_transition_M * state_vector) + (control_input_M * control_vec);
    state_covariance = (state_transition_M * (state_covariance * state_transition_M.transpose())) + process_noise_covariance;
}

void kalman_filter::update(VectorXf measurement) {
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

kalman_filter::kalman_filter(int state_dim, int control_dim, int measurement_dim, float dt) : n(state_dim), p(control_dim), m(measurement_dim), dt(dt) {
    matrix_initialize();
}

bool kalman_filter::state_initialize(VectorXf state_vec, MatrixXf state_cov) {
    bool result { false };
    if (state_vec.size() == n && state_cov.rows() == n) {
        state_vector = state_vec;
        state_covariance = state_cov;
        result = true;
    }
    return result;
}

VectorXf kalman_filter::run(VectorXf control, VectorXf measurement, float _dt) {
    if (control.size() == p && measurement.size() == m) {
        dt = _dt;
        matrix_update();
        predict(control);
        update(measurement);
    }
    return state_vector;
}

