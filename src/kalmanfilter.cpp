#include "kalmanfilter.hpp"

// Private----------------------------------------------------------------------
void KalmanFilter::matrixInit() {

    x.setZero(n);
    P.setZero(n, n); 
    F = MatrixXf::Zero(n, n);  
    B = MatrixXf::Zero(n, p);
    I = MatrixXf::Identity(n, n);
    H.setIdentity(m, n); // Setup Measurement Matrix
    Q = MatrixXf::Zero(n, n); 
    R = MatrixXf::Zero(m, m); 

    // Setup State Transition Matrix
    F << 1.0, dt, 
        0.0, 1.0;
    
    // Setup Control Input Matrix
    B << 0.5 * std::pow(dt, 2), // (Linear Displacement Eq.)
        dt; 
    
    // Setup Process-Noise Covariance
    Q(0,0) = 0.01;
    Q(1,1) = 0.1;

    // Setup Measurement Covariance
    R << 0.1;
}


void KalmanFilter::updateMatrices() {

    F(0, 1) = dt;
    B(0, 0) = 0.5 * std::pow(dt, 2);
    B(1, 0) = dt;
}


void KalmanFilter::prediction(VectorXf control_vec) {

    x = (F * x) + (B * control_vec);
    P = (F * (P * F.transpose())) + Q;
}

void KalmanFilter::update(VectorXf measurement) {

    // Innovation
    VectorXf y = measurement - (H * x);

    // Residual/Innovation Covariance
    MatrixXf S = (H * (P * H.transpose())) + R;

    // Kalman Gain
    MatrixXf K = (P * H.transpose()) * S.inverse();

    // Update
    x = x + (K * y);
    P = (I - (K * H)) * P;
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

    x = state_vec;
    P = state_cov;
    return true;
}




VectorXf KalmanFilter::run(VectorXf control, VectorXf measurement, double _dt) {
    
     if (control.size() != p || measurement.size() != m) {
        std::cout << "Error: Control Vector Size should be "<< p 
            << " Measurement Vector Size should be " << m << std::endl;
        return x;
    }

    dt = _dt;
    updateMatrices();

    prediction(control);
    update(measurement);

    return x;
}