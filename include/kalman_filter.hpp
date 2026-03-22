#pragma once

extern "C" {
#include <fix16.h>
#include <fixmatrix.h>
#include <fixkalman.h>
}

        /**
        * @brief initialize the Kalman Filter for verticality estimation
        */
        void kalman_verticality_init();

        /**
         * @brief update the Kalman Filter with new measurements
         * 
         * @param altitude current altitude measurement
         * @param vertical_acceleration current vertical acceleration measurement
         */
        void kalman_update(fix16_t altitude, fix16_t vertical_acceleration);


        /**
         * @brief calculate the drag force based on current deployment percentage and vertical velocity
         * 
         * @param deployment_percentage current deployment percentage measurement
         * @param vertical_velocity current vertical velocity measurement
         */
        fix16_t calculate_drag_force(fix16_t deployment_percentage, fix16_t vertical_velocity);

        /**
         * @brief predict the apogee based on current altitude, vertical velocity, and drag force
         * 
         * @param altitude current altitude measurement
         * @param vertical_velocity current vertical velocity measurement
         * @param drag_force current drag force measurement
         */
        fix16_t predict_apogee(fix16_t altitude, fix16_t vertical_velocity, fix16_t drag_force);

        /**
         * @brief calculate the deployment percentage based on current drag force and vertical velocity
         * 
         * @param drag_force current drag force measurement
         * @param vertical_velocity current vertical velocity measurement
         */
        fix16_t calculate_deployment_percentage(fix16_t drag_force, fix16_t vertical_velocity);

        /**
         * @brief calculate the desired drag force based on current altitude and vertical velocity
         * 
         * @param altitude current altitude measurement
         * @param vertical_velocity current vertical velocity measurement
         */
        fix16_t calculate_desired_drag_force(fix16_t altitude, fix16_t vertical_velocity);

        extern kalman16_t kf;

        extern kalman16_observation_t kfm;

        #define KALMAN_NAME verticality
        #define KALMAN_NUM_STATES 2
        #define KALMAN_NUM_INPUTS 1

        #define KALMAN_MEASUREMENT_NAME altitude
        #define KALMAN_NUM_MEASUREMENTS 1

        #define matrix_set(matrix, row, column, value) \
            matrix->data[row][column] = value

        #define matrix_set_symmetric(matrix, row, column, value) \
            matrix->data[row][column] = value; \
            matrix->data[column][row] = value

        #ifndef FIXMATRIX_MAX_SIZE
        #error FIXMATRIX_MAX_SIZE must be defined and greater or equal to the number of states, inputs and measurements.
        #endif

        #if (FIXMATRIX_MAX_SIZE < KALMAN_NUM_STATES) || (FIXMATRIX_MAX_SIZE < KALMAN_NUM_INPUTS) || (FIXMATRIX_MAX_SIZE < KALMAN_NUM_MEASUREMENTS)
        #error FIXMATRIX_MAX_SIZE must be greater or equal to the number of states, inputs and measurements.
        #endif



