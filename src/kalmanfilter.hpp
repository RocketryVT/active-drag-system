#pragma once
#include "eigen3/Eigen/Dense"
#include <iostream>
#include <cmath>

using namespace Eigen;

#define DT 1 // Timestep

class KalmanFilter {

    private:

        VectorXf x; // State Vector
        MatrixXf P; // State Covariance

        MatrixXf F; // State Transition Matrix
        MatrixXf B; // Control Input Matrix
        MatrixXf H; // Measurement Matrix

        MatrixXf Q; // Process-Noise Covariance
        MatrixXf R; // Measurement Covariance

        MatrixXf I; // Identity

        int n; // State Vector Dimension
        int p; // Control Vector Dimension
        int m; // Measurement Vector Dimension

        /**
         * @brief Initialize all necessary matrices.
         * 
         */
        void matrixInit();

        /**
         * @brief Predict current State Vector & State Covariance 
         * given the current control input.
         * 
         * @param control_vec The control input to be applied to the
         * previous State Vector
         */
        void prediction(VectorXf control_vec);

        /**
         * @brief Correct the State Vector & State Covariance predictions
         * given a related current measurement.
         * 
         * @param measurement Current measurement
         */
        void update(VectorXf measurement);

    public:

        KalmanFilter();
        
        /**
         * @brief Construct a new Kalman Filter object
         * Set the sizes of the Filter's user inputs
         * 
         * @param state_dim State Vector Dimension. i.e. dim(x)
         * @param control_dim Control/Input Vector Dimension. i.e. dim(u)
         * @param measurement_dim Measurement Vector Dimension. i.e. dim(z)
         */
        KalmanFilter(int state_dim, int control_dim, int measurement_dim);


        /**
         * @brief Optional function to set a custom initial state for the Filter.
         * If not called, State Vector & State Covariance are zero-initialized 
         * 
         * @param state_vec Initial State Vector
         * @param state_cov Initial State Covariance
         * 
         * @return Whether state initialization was successful
         */
        bool setInitialState(VectorXf state_vec, MatrixXf state_cov);

        /**
         * @brief Perform Kalman Filter operation with given control input vector
         * and measurement.
         * 
         * @param control current control command
         * @param measurement current measurement
         * 
         * @return Filtered state vector
         */
        VectorXf run(VectorXf control, VectorXf measurement);
};