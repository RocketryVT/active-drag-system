#pragma once
#include <Eigen/Dense>

using namespace Eigen;

class kalman_filter {

    private:

        VectorXf state_vector; // x
        MatrixXf state_covariance; // P

        MatrixXf state_transition_M; // F
        MatrixXf control_input_M; // B
        MatrixXf measurement_M; // H

        MatrixXf process_noise_covariance; // Q
        MatrixXf measurement_covariance; // R

        MatrixXf I; // Identity

        int n; // State Vector Dimension
        int p; // Control Vector Dimension
        int m; // Measurement Vector Dimension

        float dt; // timestep

        /**
         * @brief Initialize all necessary matrices.
         * 
         */
        void matrix_initialize();

        /**
         * @brief Update any existing variable elements in your State Transition 
         * & Control Input matrices.
         * 
         */
        void matrix_update();

        /**
         * @brief Predict current State Vector & State Covariance 
         * given the current control input.
         * 
         * @param control_vec The control input to be applied to the
         * previous State Vector
         */
        void predict(VectorXf control_vec);

        /**
         * @brief Correct the State Vector & State Covariance predictions
         * given a related current measurement.
         * 
         * @param measurement Current measurement
         */
        void update(VectorXf measurement);

    public:

        kalman_filter();
        
        /**
         * @brief Construct a new Kalman Filter object
         * Set the sizes of the Filter's user inputs
         * 
         * @param state_dim State Vector Dimension. i.e. dim(x)
         * @param control_dim Control/Input Vector Dimension. i.e. dim(u)
         * @param measurement_dim Measurement Vector Dimension. i.e. dim(z)
         * @param dt timestep
         */
        kalman_filter(int state_dim, int control_dim, int measurement_dim, float dt);


        /**
         * @brief Optional function to set a custom initial state for the Filter.
         * If not called, State Vector & State Covariance are zero-initialized 
         * 
         * @param state_vec Initial State Vector
         * @param state_cov Initial State Covariance
         * 
         * @return Whether state initialization was successful
         */
        bool state_initialize(VectorXf state_vec, MatrixXf state_cov);

        /**
         * @brief Perform Kalman Filter operation with given control input vector
         * and measurement.
         * 
         * @param control current control command
         * @param measurement current measurement
         * @param dt timestep
         * 
         * @return Filtered state vector
         */
        VectorXf run(VectorXf control, VectorXf measurement, float _dt);
};
